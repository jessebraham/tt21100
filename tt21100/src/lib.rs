//! TT21100 Multi-Touch Touchscreen Controller

#![no_std]

use core::{array::TryFromSliceError, fmt::Debug};

use bondrewd::Bitfields;
use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
    digital::v2::InputPin,
};

// Default I²C address for the TT21100
const I2C_ADDR: u8 = 0x24;

/// Any type of error which may occur while interacting with the device
#[derive(Debug)]
pub enum Error<E> {
    /// Some error originating from the communication bus
    BusError(E),
    /// The message length did not match the expected value
    InvalidMessageLen(usize),
    /// Reading a GPIO pin resulted in an error
    IOError,
    /// Tried to read a touch point, but no data was available
    NoDataAvailable,
    /// Error converting a slice to an array
    TryFromSliceError,
}

impl<E> From<TryFromSliceError> for Error<E> {
    fn from(_: TryFromSliceError) -> Self {
        Self::TryFromSliceError
    }
}

/// An event emitted by the device
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// A touch event
    Touch {
        report: TouchReport,
        touches: (Option<TouchRecord>, Option<TouchRecord>),
    },
    /// A button press event
    Button(ButtonRecord),
}

/// Prelude data for one or more touch events
#[derive(Debug, Clone, Copy, PartialEq, Eq, Bitfields)]
#[bondrewd(default_endianness = "le")]
pub struct TouchReport {
    /// Total length of the data; should be 7, 17, or 27
    pub data_len: u16,
    /// ID of the report
    pub report_id: u8,
    /// Timestamp
    pub time_stamp: u16,
    #[bondrewd(bit_length = 2)]
    padding0: u8,
    #[bondrewd(bit_length = 1)]
    pub large_object: u8,
    #[bondrewd(bit_length = 5)]
    pub record_num: u8,
    #[bondrewd(bit_length = 2)]
    pub report_counter: u8,
    #[bondrewd(bit_length = 3)]
    padding1: u8,
    #[bondrewd(bit_length = 3)]
    pub noise_effect: u8,
}

/// Data for a touch event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Bitfields)]
#[bondrewd(default_endianness = "le")]
pub struct TouchRecord {
    #[bondrewd(bit_length = 5)]
    padding0: u8,
    #[bondrewd(bit_length = 3)]
    pub touch_type: u8,
    #[bondrewd(bit_length = 1)]
    pub tip: u8,
    #[bondrewd(bit_length = 2)]
    pub event_id: u8,
    #[bondrewd(bit_length = 5)]
    pub touch_id: u8,
    pub x: u16,
    pub y: u16,
    pub pressure: u8,
    pub major_axis_length: u16,
    pub orientation: u8,
}

/// Data for a button press event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Bitfields)]
#[bondrewd(default_endianness = "le")]
pub struct ButtonRecord {
    /// Length of the record; always `14`
    pub length: u16,
    /// ID of the report; always `3`
    pub report_id: u8,
    /// Timestamp in units of 100us
    pub time_stamp: u16,
    /// Button value; only use bits[3..0]
    pub btn_val: u8,
    /// Button signals
    pub btn_signal: [u16; 4],
}

/// TT21100 driver
pub struct TT21100<I2C, IRQ> {
    /// Underlying I²C peripheral
    i2c: I2C,
    /// Interrupt pin
    irq: IRQ,
}

impl<I2C, IRQ, E> TT21100<I2C, IRQ>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
    IRQ: InputPin,
    E: Debug,
{
    /// Create a new instance of the driver and initialize the device
    pub fn new(i2c: I2C, irq: IRQ) -> Result<Self, Error<E>> {
        let mut me = Self { i2c, irq };

        // I'm honestly not entirely sure what is going on here (would be *really* nice
        // if I had a datasheet!).
        //
        // As far as I can tell, when no events are queued on device for reading it will
        // always return an empty message with length 2, so we're just sort of
        // making sure we can talk to the device.
        //
        // Each driver I referenced seems to perform this step:
        //
        // https://github.com/espressif/esp-box/blob/147cd8d/components/i2c_devices/touch_panel/tt21100.c#L56-L60
        // https://github.com/SuGlider/Adafruit_ESP32S3_BOX/blob/a9884ac/src/ESP32_S3_Box_TouchScreen.cpp#L15-L20
        // https://github.com/adafruit/Adafruit_CircuitPython_TT21100/blob/b3113a4/adafruit_tt21100.py#L59-L63
        let mut message_length = 0;
        for _ in 0..5 {
            message_length = me.read_message_length()?;
            if message_length == 2 {
                break;
            }
        }

        match message_length {
            2 => Ok(me),
            n => Err(Error::InvalidMessageLen(n)),
        }
    }

    /// Is there data available to read from the device?
    pub fn data_available(&self) -> Result<bool, Error<E>> {
        self.irq.is_low().map_err(|_| Error::IOError)
    }

    /// Read an event from the device
    ///
    /// There are two types of events, [Event::Touch] and [Event::Button].
    pub fn event(&mut self) -> Result<Event, Error<E>> {
        let message_length = self.read_message_length()?;

        let mut data = [0u8; 32];
        self.read_bytes(&mut data[0..][..message_length])?;

        match message_length {
            2 => Err(Error::NoDataAvailable),
            7 | 17 | 27 => touch_event(&data[0..][..message_length]),
            14 => button_event(&data[0..][..message_length]),
            n => Err(Error::InvalidMessageLen(n)),
        }
    }

    // -----------------------------------------------------------------------
    // PRIVATE

    fn read_message_length(&mut self) -> Result<usize, Error<E>> {
        let mut buffer = [0u8; 2];
        self.read_bytes(&mut buffer)?;

        let message_length = u16::from_le_bytes(buffer);

        Ok(message_length as usize)
    }

    fn read_bytes(&mut self, buffer: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(I2C_ADDR, &[], buffer)
            .map_err(|e| Error::BusError(e))
    }
}

fn touch_event<E>(message: &[u8]) -> Result<Event, Error<E>>
where
    E: Debug,
{
    debug_assert!(message.len() == 7 || message.len() == 17 || message.len() == 27);

    let report = message[0..][..7].try_into()?;
    let report = TouchReport::from_bytes(report);

    let record0 = if message.len() >= 17 {
        let record = message[7..][..10].try_into()?;
        let record = TouchRecord::from_bytes(record);

        Some(record)
    } else {
        None
    };

    let record1 = if message.len() == 27 {
        let record = message[17..][..10].try_into()?;
        let record = TouchRecord::from_bytes(record);

        Some(record)
    } else {
        None
    };

    Ok(Event::Touch {
        report,
        touches: (record0, record1),
    })
}

fn button_event<E>(message: &[u8]) -> Result<Event, Error<E>>
where
    E: Debug,
{
    debug_assert_eq!(message.len(), 14);

    let message = message.try_into()?;
    let record = ButtonRecord::from_bytes(message);

    Ok(Event::Button(record))
}
