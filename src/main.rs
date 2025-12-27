pub mod protocol;
pub mod serial_utils;

use crate::protocol::VnSimulData;
use std::thread;
use std::time::{Duration, Instant};
use serialport::SerialPort;




fn main() {
    

    println!("Hello, world!");

    
    let frequency: f32 = 114.0; //Hz
    let interval = Duration::from_secs_f32(1.0 / frequency);

    
    let mut port: Box<dyn SerialPort> = serial_utils::get_serial_port(921_600); //open tty

    let mut reader =
        csv::Reader::from_path("simulation_data.csv").expect("Failed to open CSV file"); //open csv

    println!("Starting HIL Simulation at {}Hz...", frequency);

    for result in reader.deserialize() {
        let start_time = Instant::now();

        let row: VnSimulData= result.expect("CSV format error");
        let packet = row.packet_to_vn();
        

        //Write to the serial port
        port.write_all(&packet).expect("Serial write failed");
        let elapsed = start_time.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
        
    }
}
