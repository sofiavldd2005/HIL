//! # Protocol Module
//! This module handles the mapping between CSV simulation data
//! and the VectorNav binary serial protocol.
//! This module also handles the data output of the stm into the dedicated CSVs
//! 

/// Offset calculation: 1 (Sync) + 1 (ID) + SAFE_BITS_LEN
const PAYLOAD_OFFSET: usize = 2; // Example: Sync(1) + ID(1) 
use serde::{Deserialize, Serialize};
/// deserialize to directly map from csv to struct : fields as written in the CSV
///strut read from the csv and to be sent via serial port (excluding the headers and CRCs)
///
/// ```c char array : [8] to [127]```
///### VectorNav IMU Structure Definition
/// 
/// This C structure represents the binary data layout for the IMU messages.
///
/// ```c
/// typedef struct vn_output_msg_test {
///     // sd card stuff
///     uint8_t safe_bits[STRUCT_SAFE_BITS_LEN];
///
///     // char array : [0] a [129]
///     // first 8 bytes are headers corresponding to the groups
///     float yaw;                    // deg   [8]-[11]
///     float pitch;                  // deg   [12]-[15]
///     float roll;                   // deg   [16]-[19]
///
///     // m/s² (ou seja sem passar pelos filtros do vn)
///     float uncomp_accel_x;         // [20]-[23]
///     float uncomp_accel_y;         // [24]-[27]
///     float uncomp_accel_z;         // [28]-[31]
///
///     // rad/s (ou seja sem passar pelos filtros do vn)
///     float uncomp_angular_rate_x;  // [32]-[35]
///     float uncomp_angular_rate_y;  // [36]-[39]
///     float uncomp_angular_rate_z;  // [40]-[43]
///
///     float temp;                   // ºC    [44]-[47]
///     float press;                  // kPa   [48]-[51]
///
///     float mag_x;                  // gauss [52]-[55]
///     float mag_y;                  // gauss [56]-[59]
///     float mag_z;                  // gauss [60]-[63]
///
///     // m/s² (ou seja, ja passando pelos filtros)
///     float accel_x;                // [64]-[67]
///     float accel_y;                // [68]-[71]
///     float accel_z;                // [72]-[75]
///
///     // rad/s
///     float angular_rate_x;         // [76]-[79]
///     float angular_rate_y;         // [80]-[83]
///     float angular_rate_z;         // [84]-[87]
///
///     float quat_x;                 // [88]-[91]
///     float quat_y;                 // [92]-[95]
///     float quat_z;                 // [96]-[99]
///     float quat_w;                 // [100]-[103]
///
///     float lin_accel_x;            // [104]-[107]
///     float lin_accel_y;            // [108]-[111]
///     float lin_accel_z;            // [112]-[115]
///
///     float accel_N;                // [116]-[119]
///     float accel_E;                // [120]-[123]
///     float accel_D;                // [124]-[127]
///
///     // CRC [128]-[129]
///
///     // other stuff
///     float accel_according_to_gravity;
///     float altitude;
///     uint32_t time;
/// } vectornav_imu_typedef;
/// ```
/// 

#[derive(Debug, Deserialize)]
pub struct VnSimulData {
    pub Yaw: f32,
    pub Pitch: f32,
    pub Roll: f32,
    pub UncompAccX: f32,
    pub UncompAccY: f32,
    pub UncompAccZ: f32,
    pub UncompGyroX: f32,
    pub UncompGyroY: f32,
    pub UncompGyroZ: f32,
    pub Temperature: f32,
    pub Pressure: f32,
    pub MagX: f32,
    pub MagY: f32,
    pub MagZ: f32,
    pub AccelX: f32,
    pub AccelY: f32,
    pub AccelZ: f32,
    pub GyroX: f32,
    pub GyroY: f32,
    pub GyroZ: f32,
    pub QuatX: f32,
    pub QuatY: f32,
    pub QuatZ: f32,
    pub QuatS: f32,
    pub LinAccelX: f32,
    pub LinAccelY: f32,
    pub LinAccelZ: f32,
    pub LinAccelN: f32,
    pub LinAccelE: f32,
    pub LinAccelD: f32,
}


///This struct shall save the output of the vertical kalman filter that runs on the stm. 
/// 
/// The serialize trait is aplied because we want to save the data to a CSV later
/// 
///### STM Output data struct
/// 
/// This C structure represents the binary data layout for the IMU messages.
/// ```c
/// Define Packet IDs
/// 
/// #define ID_VERTICAL   0x01  // 100Hz 
/// #define ID_HORIZONTAL 0x02  // 10Hz
/// #define ID_CONTROLLER 0x03  // 10Hz (or combined with horizontal)
/// typedef struct __attribute__((packed)) {
///     uint8_t sync_byte; // 0xAA
///    uint8_t packet_id; // ID
///     uint8_t payload[64]; // Size largest msg possible
///     uint16_t crc;
/// } OutgoingPacket_t;
/// ```
#[derive(Debug, Serialize)]
pub struct VerticalFilterOutput {
    pub altitude: f32,
    pub velocity: f32,
    pub acceleration: f32,
}
///This struct shall save the output of the horizontal kalman filter that runs on the stm. 
/// The serialize trait is aplied because we want to save the data to a CSV later
#[derive(Debug, Serialize)]
pub struct HorizontalFilterOutput {
    pub mag: [f32; 3],
    pub pos: [f32; 3],
    pub vel: [f32; 3],
    pub acc: [f32; 3],
}

///This struct shall save the output of the  airbrakes controler that runs on the stm. 
/// The serialize trait is aplied because we want to save the data to a CSV later
pub struct AirbakesControlerOutput{
    pub apogee_prediction : f32,
    pub airbrakes_opening : f32,
}

/// we use this to convert the data from the structure into a X-byte VectorNav binary packet
impl VnSimulData {
    /// Converts the structure data into a 130-byte VectorNav binary packet.
    ///
    /// The first 8 bytes are headers (Sync, Group selection, etc.).
    /// See the VN Control Center to verify these header bytes.
    ///
    /// # Return
    /// Returns a `Vec<u8>` of length 130 ready for serial transmission.
    ///
    pub fn packet_to_vn(&self) -> Vec<u8> {
        // to initialize the binary array to be sent via the serial port

        let mut packet: Vec<u8> = vec![0; 130]; //initialize an array to zeros with the vec! macro
        //Initalize headers -- see Vn Control center
        packet[0] = 0xFA; //sync bit
        packet[1] = 0x15;
        packet[2] = 0x08;
        packet[3] = 0x02;
        packet[4] = 0x30;
        packet[5] = 0x07;
        packet[6] = 0xC4;
        packet[7] = 0x00;

        // Payload

        // Euler Angles [8-19]
        packet[8..12].copy_from_slice(&self.Yaw.to_le_bytes());
        packet[12..16].copy_from_slice(&self.Pitch.to_le_bytes());
        packet[16..20].copy_from_slice(&self.Roll.to_le_bytes());

        // Uncomp Accel [20-31]
        packet[20..24].copy_from_slice(&self.UncompAccX.to_le_bytes());
        packet[24..28].copy_from_slice(&self.UncompAccY.to_le_bytes());
        packet[28..32].copy_from_slice(&self.UncompAccZ.to_le_bytes());

        // Uncomp Gyro [32-43]
        packet[32..36].copy_from_slice(&self.UncompGyroX.to_le_bytes());
        packet[36..40].copy_from_slice(&self.UncompGyroY.to_le_bytes());
        packet[40..44].copy_from_slice(&self.UncompGyroZ.to_le_bytes());

        // Temperature & Pressure [44-51]
        packet[44..48].copy_from_slice(&self.Temperature.to_le_bytes());
        packet[48..52].copy_from_slice(&self.Pressure.to_le_bytes());

        // Mag [52-63]
        packet[52..56].copy_from_slice(&self.MagX.to_le_bytes());
        packet[56..60].copy_from_slice(&self.MagY.to_le_bytes());
        packet[60..64].copy_from_slice(&self.MagZ.to_le_bytes());

        // Accel [64-75]
        packet[64..68].copy_from_slice(&self.AccelX.to_le_bytes());
        packet[68..72].copy_from_slice(&self.AccelY.to_le_bytes());
        packet[72..76].copy_from_slice(&self.AccelZ.to_le_bytes());

        // Gyro [76-87]
        packet[76..80].copy_from_slice(&self.GyroX.to_le_bytes());
        packet[80..84].copy_from_slice(&self.GyroY.to_le_bytes());
        packet[84..88].copy_from_slice(&self.GyroZ.to_le_bytes());

        // Quat [88-103]
        packet[88..92].copy_from_slice(&self.QuatX.to_le_bytes());
        packet[92..96].copy_from_slice(&self.QuatY.to_le_bytes());
        packet[96..100].copy_from_slice(&self.QuatZ.to_le_bytes());
        packet[100..104].copy_from_slice(&self.QuatS.to_le_bytes());
        // Lin Accel [104-115]
        packet[104..108].copy_from_slice(&self.LinAccelX.to_le_bytes());
        packet[108..112].copy_from_slice(&self.LinAccelY.to_le_bytes());
        packet[112..116].copy_from_slice(&self.LinAccelZ.to_le_bytes());

        // Accel NED [116-127]
        packet[116..120].copy_from_slice(&self.LinAccelN.to_le_bytes());
        packet[120..124].copy_from_slice(&self.LinAccelE.to_le_bytes());
        packet[124..128].copy_from_slice(&self.LinAccelD.to_le_bytes());

        //crc [128-129];
        //let crc = calculate_crc(&p);
        // packet[128..130].copy_from_slice(&crc.to_le_bytes());
        //packet[128] = 0xCC; // Dummy CRC
        // packet[129] = 0xAA; 
        
        // Calculate CRC using the VectorNav algorithm
        let checksum = calculate_vn_crc(&packet);
        let crc_bytes = checksum.to_le_bytes();
        packet[128] = crc_bytes[0];
        packet[129] = crc_bytes[1];
        packet
    }
}


/// Rust implementation of the VectorNav Datasheet CRC
pub fn calculate_vn_crc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    // We start at index 1 to skip the sync byte '0xFA' as per your C code
    for i in 1..128 {
        let byte = data[i];
        crc = ((crc >> 8) | (crc << 8)) & 0xFFFF;
        crc ^= byte as u16;
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }
    crc
}

// --- Helper Functions for Serial Parsing ---

///Checks if calculated CRC is equal to received CRC
pub fn is_packet_valid(buffer: &[u8]) -> bool {
    if buffer.len() < 130 { return false; }
    let received_crc = u16::from_le_bytes([buffer[128], buffer[129]]);
    calculate_vn_crc(buffer) == received_crc
}



pub fn parse_vertical(bytes: &[u8]) -> VerticalFilterOutput {
    // Matches: safe_bits, altitude, velocity, acceleration, time
    VerticalFilterOutput {
        altitude: f32::from_le_bytes(bytes[PAYLOAD_OFFSET..PAYLOAD_OFFSET+4].try_into().unwrap()),
        velocity: f32::from_le_bytes(bytes[PAYLOAD_OFFSET+4..PAYLOAD_OFFSET+8].try_into().unwrap()),
        acceleration: f32::from_le_bytes(bytes[PAYLOAD_OFFSET+8..PAYLOAD_OFFSET+12].try_into().unwrap()),
    }
}

pub fn parse_horizontal(bytes: &[u8]) -> HorizontalFilterOutput {
    // Matches arrays: mag[3], pos[3], vel[3], acc[3], time
    let mut o = PAYLOAD_OFFSET;
    
    let mut read_3f32 = |offset: &mut usize| {
        let arr = [
            f32::from_le_bytes(bytes[*offset..*offset+4].try_into().unwrap()),
            f32::from_le_bytes(bytes[*offset+4..*offset+8].try_into().unwrap()),
            f32::from_le_bytes(bytes[*offset+8..*offset+12].try_into().unwrap()),
        ];
        *offset += 12;
        arr
    };

    HorizontalFilterOutput {
        mag: read_3f32(&mut o),
        pos: read_3f32(&mut o),
        vel: read_3f32(&mut o),
        acc: read_3f32(&mut o),
    }
}

pub fn parse_controller(bytes: &[u8]) -> AirbakesControlerOutput {
    // Matches: safe_bits, apogee, airbrakes, time
    AirbakesControlerOutput {
        apogee_prediction: f32::from_le_bytes(bytes[PAYLOAD_OFFSET..PAYLOAD_OFFSET+4].try_into().unwrap()),
        airbrakes_opening: f32::from_le_bytes(bytes[PAYLOAD_OFFSET+4..PAYLOAD_OFFSET+8].try_into().unwrap()),

    }
}