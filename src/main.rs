mod protocol;
mod serial_utils;

use crate::protocol::Vn_simul_data;
use std::thread;
use std::time::{Duration, Instant};
use serialport::SerialPort;




fn main() {
    /*
    In Rust, the SerialPort object has a Drop implementation.
    When main finishes, Rust identifies that port is no longer needed and runs its internal "cleanup" code automatically,
    which releases the system handle for /dev/ttyUSB
     */
    println!("Hello, world!");

    
    let frequency: f32 = 114.0; //Hz
    let interval = Duration::from_secs_f32(1.0 / frequency);

    ///Open serial port
    let mut port: Box<dyn SerialPort> = serial_utils::get_serial_port(921_600); //open tty

    let mut reader =
        csv::Reader::from_path("simulation_data.csv").expect("Failed to open CSV file"); //open csv

    let frequency: f32 = 114.0; //Hz
    let interval = Duration::from_secs_f32(1.0 / frequency);

    println!("Starting HIL Simulation at {}Hz...", frequency);

    for result in reader.deserialize() {
        let start_time = Instant::now();

        let row: Vn_simul_data= result.expect("CSV format error");
        let packet = row.packet_to_vn();

        port.write_all(&packet).expect("Serial write failed");
        let elapsed = start_time.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
        println!("Finished streaming CSV. Port closed.");
    }
}
