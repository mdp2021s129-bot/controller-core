/// Command encoding & result parsing for the DFRobot URM37 ultrasonic distance
/// sensor.

/// Communication between the sensor and the microcontroller takes place in
/// units of 4 byte `Frame`s.
pub type Frame = [u8; 4];

/// Length of all command and reply byte sequences, in bytes.
pub const COMMAND_LENGTH: usize = 4;
/// Command used to retrieve the mode of the sensor.
pub const RETRIEVE_MODE_COMMAND: Frame = [0x33, 0x02, 0xff, 0x34];
/// Command used to change the mode of the sensor to the passive (triggered)
/// mode.
pub const CHANGE_ONESHOT_MODE_COMMAND: Frame = [0x44, 0x02, 0xbb, 0x01];
/// Command used to request a distance measurement from the sensor.
pub const DISTANCE_MEASUREMENT_START_COMMAND: Frame = [0x22, 0x0f, 0xf0, 0x21];

/// Sensor operation modes.
pub enum Mode {
    /// Oneshot operation mode. A reading must be triggered from either the
    /// control pin or the serial interface.
    Oneshot,
    /// Continuous operation mode. Readings are taken by the sensor
    /// automatically.
    Continuous,
}

/// Calculate the checksum of a sequence of bytes using the algorithm as
/// mentioned in the sensor's datasheet.
///
/// Simply a sum across all the bytes.
fn checksum(data: &[u8]) -> u8 {
    data.iter().sum()
}

/// Validate the checksum of a sequence of data bytes against a precalculated
/// value.
///
/// Uses the algorithm as mentioned in the sensor's datasheet.
fn validate_checksum(frame: Frame) -> bool {
    checksum(&frame[..3]) == frame[3]
}

/// Mode parse error.
#[derive(Clone, Debug)]
pub enum ModeError {
    /// Sensor is operating in an unknown mode.
    Unknown,
    /// Checksum error.
    Checksum,
}

/// Parse a mode reply.
pub fn parse_mode(reply: Frame) -> Result<Mode, ModeError> {
    if !validate_checksum(reply) {
        return Err(ModeError::Checksum);
    }

    match reply[1] {
        0xaa => Ok(Mode::Continuous),
        0xbb => Ok(Mode::Oneshot),
        _ => Err(ModeError::Unknown),
    }
}

/// Distance parse error.
#[derive(Clone, Debug)]
pub enum DistanceError {
    /// Sensor couldn't determine distance.
    Indeterminate,
    /// Checksum error.
    Checksum,
}

/// Parse a distance reply.
///
/// If valid, returns the distance in centimeters.
pub fn parse_distance(reply: Frame) -> Result<u16, DistanceError> {
    if !validate_checksum(reply) {
        return Err(DistanceError::Checksum);
    }

    let distance_bytes = [reply[2], reply[1]];
    // Avoids `TryInto`.
    let distance = u16::from_le_bytes(distance_bytes);

    if distance == 0xffff {
        Err(DistanceError::Indeterminate)
    } else {
        Ok(distance)
    }
}
