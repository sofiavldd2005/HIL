mod protocol;
mod serial_utils;

use crate::protocol::{
    AirbakesControlerOutput, HorizontalFilterOutput, VerticalFilterOutput, VnSimulData,
};
use serialport::SerialPort;
use std::mem::size_of;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};
fn main() {
    // 1. Initialize Serial Port
    let port = serial_utils::get_serial_port(921_600);

    // Clone for bidirectional communication
    let mut reader = port.try_clone().expect("Failed to clone reader");
    let mut writer = port;

    // 2. Setup Shutdown Signal
    // Arc allows both threads to own the same 'running' flag
    let running = Arc::new(AtomicBool::new(true));
    let r_running = Arc::clone(&running);

    // 3. Telemetry Receiver Thread (STM32 -> PC)
    let receiver_handle = thread::spawn(move || {
        run_receiver(reader, r_running);
    });
    // 4. Main Sender Loop (PC -> STM32 @ 114Hz)
    let mut csv_reader = csv::Reader::from_path("simulation_data.csv").expect("CSV missing");
    let interval = Duration::from_secs_f32(1.0 / 114.0);

    println!("Starting simulation...");
    for result in csv_reader.deserialize() {
        let start = Instant::now();
        let row: VnSimulData = result.expect("CSV error");

        // Send 130-byte packet
        writer.write_all(&row.packet_to_vn()).expect("Write error");

        let elapsed = start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    // 5. Cleanup
    println!("Simulation finished.");
    running.store(false, Ordering::SeqCst); // Signal the receiver thread to stop
    receiver_handle
        .join()
        .expect("Receiver thread failed to close cleanly");
    println!("HIL system closed safely.");
}

pub fn process_stack(
    receive_stack: &mut Vec<u8>,
    wtr_vert: &mut csv::Writer<std::fs::File>,
    wtr_horiz: &mut csv::Writer<std::fs::File>,
    wtr_ctrl: &mut csv::Writer<std::fs::File>,
) {
    // We need at least 2 bytes (Sync + ID) to start processing
    while receive_stack.len() >= 2 {
        if receive_stack[0] == 0xAA {
            // Sync byte
            let packet_id = receive_stack[1];

            // Calculate length dynamically based on structure size
            // Note: We add 2 bytes for the header (Sync/ID)
            let packet_len: usize = match packet_id {
                0x01 => 2 + size_of::<VerticalFilterOutput>(),
                0x02 => 2 + size_of::<HorizontalFilterOutput>(),
                0x03 => 2 + size_of::<AirbakesControlerOutput>(),
                _ => {
                    // Unknown ID: remove corrupt sync byte and keep hunting
                    receive_stack.remove(0);
                    continue;
                }
            };

            // Check if the full packet has arrived
            if receive_stack.len() >= packet_len {
                // Extract the exact number of bytes for this packet
                let packet: Vec<u8> = receive_stack.drain(..packet_len).collect();

                // Validate the VectorNav CRC
                if protocol::is_packet_valid(&packet) {
                    match packet_id {
                        0x01 => {
                            let data = protocol::parse_vertical(&packet);
                            wtr_vert.serialize(data).unwrap();
                        }
                        0x02 => {
                            let data = protocol::parse_horizontal(&packet);
                            wtr_horiz.serialize(data).unwrap();
                        }
                        0x03 => {
                            let data = protocol::parse_controller(&packet);
                            wtr_ctrl.serialize(data).unwrap();
                        }
                        _ => {}
                    }
                }
            } else {
                // Full packet not yet received, wait for more data
                break;
            }
        } else {
            // Not a sync byte, remove it and keep looking
            receive_stack.remove(0);
        }
    }
}

/// Dedicated function for the background receiver logic
fn run_receiver(mut reader: Box<dyn SerialPort>, running: Arc<AtomicBool>) {
    let mut receive_stack: Vec<u8> = Vec::new();
    let mut serial_buf: Vec<u8> = vec![0; 1024];

    let mut wtr_vert = csv::Writer::from_path("vertical_100hz.csv").unwrap();
    let mut wtr_horiz = csv::Writer::from_path("horizontal_10hz.csv").unwrap();
    let mut wtr_ctrl = csv::Writer::from_path("controller_10hz.csv").unwrap();

    while running.load(Ordering::SeqCst) || !receive_stack.is_empty() {
        if let Ok(n) = reader.read(serial_buf.as_mut_slice()) {
            receive_stack.extend_from_slice(&serial_buf[..n]);
        } else if !running.load(Ordering::SeqCst) {
            break;
        }

        // Use the dynamic size_of matching logic here
        process_stack(
            &mut receive_stack,
            &mut wtr_vert,
            &mut wtr_horiz,
            &mut wtr_ctrl,
        );
    }

    // Final flush
    // After the loop finishes, we force the buffers to disk
    wtr_vert.flush().unwrap();
    wtr_horiz.flush().unwrap();
    wtr_ctrl.flush().unwrap();
}
