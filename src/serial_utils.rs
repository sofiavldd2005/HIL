//! # Serial utlis  Module
//! This module handles opening the serial port (ttyACM, we are using the ST-Link as the bridge between the PC and the STM)
use serialport::SerialPort;

///Open serial port
///In Rust, the SerialPort object has a Drop implementation.
///When main finishes, Rust identifies that port is no longer needed and runs its internal "cleanup" code automatically,
///which releases the system handle for /dev/ttyUSB
pub fn get_serial_port(baud: u32) -> Box<dyn serialport::SerialPort> {
    let ports = serialport::available_ports().expect("No ports found");
    let port_name = ports
        .iter()
        .find(|p| p.port_name.contains("ttyACM") || p.port_name.contains("ttyUSB"))
        .map(|p| p.port_name.clone())
        .expect("No suitable USB-Serial device found");
    println!("Connecting to {} at {} baud...", port_name, baud);
    serialport::new(port_name, baud)
        .timeout(std::time::Duration::from_millis(100))
        .open()
        .expect("Failed to open port")
}
