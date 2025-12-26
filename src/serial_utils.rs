use serialport::{SerialPort, SerialPortType};
use std::time::Duration;


pub fn get_serial_port(baud: u32) -> Box<dyn serialport::SerialPort> {
    let ports = serialport::available_ports().expect("No ports found");
    let port_name = ports.iter()
        .find(|p| p.port_name.contains("ttyACM") || p.port_name.contains("ttyUSB"))
        .map(|p| p.port_name.clone())
        .expect("No suitable USB-Serial device found");

    serialport::new(port_name, baud)
        .timeout(std::time::Duration::from_millis(100))
        .open()
        .expect("Failed to open port")
}