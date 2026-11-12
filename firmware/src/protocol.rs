//! Newline-delimited cyberlamp console commands, independent of the BLE transport.
//!
//! Commands are two letters, so they are quick to type in a serial terminal:
//! `br 20`, `mx 20`, `md wh|sp|uv|ir`, `st`, `he`. The original long names
//! (`brightness`, `mix`, `mode white`, `status`, `help`) are still accepted.

use core::fmt;

pub const MAX_LEVEL: u8 = 40;
pub const MAX_LINE: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Brightness(u8),
    Mix(u8),
    Mode(u8), // settings encoding: white=0, uv=1, ir=2, spread=3
    Status,
    Help,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandError {
    Text,
    Arguments,
    Range,
    Unknown,
    TooLong,
}

impl CommandError {
    pub fn message(self) -> &'static str {
        match self {
            Self::Text => "ERR invalid text\n",
            Self::Arguments => "ERR arguments; use he\n",
            Self::Range => "ERR range 0..40\n",
            Self::Unknown => "ERR unknown command; use he\n",
            Self::TooLong => "ERR line too long\n",
        }
    }
}

fn parse(line: &[u8]) -> Result<Command, CommandError> {
    let text = core::str::from_utf8(line).map_err(|_| CommandError::Text)?;
    let mut words = text.split_ascii_whitespace();
    let name = words.next().ok_or(CommandError::Arguments)?;
    let argument = words.next();
    if words.next().is_some() {
        return Err(CommandError::Arguments);
    }
    match name {
        "st" | "status" if argument.is_none() => Ok(Command::Status),
        "he" | "help" if argument.is_none() => Ok(Command::Help),
        "br" | "brightness" | "mx" | "mix" => {
            let level = argument
                .ok_or(CommandError::Arguments)?
                .parse::<u8>()
                .map_err(|_| CommandError::Range)?;
            if level > MAX_LEVEL {
                return Err(CommandError::Range);
            }
            Ok(if matches!(name, "br" | "brightness") {
                Command::Brightness(level)
            } else {
                Command::Mix(level)
            })
        }
        "md" | "mode" => match argument {
            Some("wh" | "white") => Ok(Command::Mode(0)),
            Some("uv") => Ok(Command::Mode(1)),
            Some("ir") => Ok(Command::Mode(2)),
            Some("sp" | "spread") => Ok(Command::Mode(3)),
            _ => Err(CommandError::Arguments),
        },
        "st" | "status" | "he" | "help" => Err(CommandError::Arguments),
        _ => Err(CommandError::Unknown),
    }
}

pub struct Decoder {
    line: [u8; MAX_LINE],
    len: usize,
    overflow: bool,
}

impl Decoder {
    pub const fn new() -> Self {
        Self {
            line: [0; MAX_LINE],
            len: 0,
            overflow: false,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.overflow = false;
    }

    /// An overlong line is discarded in its entirety, never executed as a prefix.
    pub fn push(&mut self, byte: u8) -> Option<Result<Command, CommandError>> {
        if byte == b'\n' || byte == b'\r' {
            let result = if self.overflow {
                Some(Err(CommandError::TooLong))
            } else if self.line[..self.len].iter().all(u8::is_ascii_whitespace) {
                None
            } else {
                Some(parse(&self.line[..self.len]))
            };
            self.reset();
            return result;
        }
        if self.len == MAX_LINE {
            self.overflow = true;
        }
        if !self.overflow {
            self.line[self.len] = byte;
            self.len += 1;
        }
        None
    }
}

/// Bounded reply storage, moved between tasks without borrowing stack buffers.
pub struct Reply {
    data: [u8; 128],
    len: usize,
}

impl Reply {
    pub fn new() -> Self {
        Self { data: [0; 128], len: 0 }
    }
    pub fn bytes(&self) -> &[u8] {
        &self.data[..self.len]
    }
}

impl fmt::Write for Reply {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if self.len + text.len() > self.data.len() {
            return Err(fmt::Error);
        }
        self.data[self.len..self.len + text.len()].copy_from_slice(text.as_bytes());
        self.len += text.len();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framing_survives_fragmented_writes_and_crlf() {
        let mut d = Decoder::new();
        let mut expected = [Ok(Command::Brightness(20)), Ok(Command::Status), Ok(Command::Help)].into_iter();
        for chunk in [b"bright".as_slice(), b"ness 20\r\nsta", b"tus\nhelp\n"] {
            for &byte in chunk {
                if let Some(command) = d.push(byte) {
                    assert_eq!(Some(command), expected.next());
                }
            }
        }
        assert_eq!(expected.next(), None);
    }

    #[test]
    fn rejects_bad_commands_and_out_of_range_values() {
        for line in [b"brightness 41".as_slice(), b"mix -1", b"mix 256"] {
            assert_eq!(parse(line), Err(CommandError::Range));
        }
        assert_eq!(parse(b"mode violet"), Err(CommandError::Arguments));
        assert_eq!(parse(b"status extra"), Err(CommandError::Arguments));
        assert_eq!(parse(b"mix 10 extra"), Err(CommandError::Arguments));
        assert_eq!(parse(b"\xff"), Err(CommandError::Text));
        assert_eq!(parse(b"reboot"), Err(CommandError::Unknown));
        assert_eq!(parse(b" brightness 0 "), Ok(Command::Brightness(0)));
        assert_eq!(parse(b"mix 40"), Ok(Command::Mix(40)));
        assert_eq!(parse(b"mode ir"), Ok(Command::Mode(2)));
    }

    #[test]
    fn two_letter_commands() {
        assert_eq!(parse(b"br 20"), Ok(Command::Brightness(20)));
        assert_eq!(parse(b"mx 0"), Ok(Command::Mix(0)));
        assert_eq!(parse(b"md wh"), Ok(Command::Mode(0)));
        assert_eq!(parse(b"md uv"), Ok(Command::Mode(1)));
        assert_eq!(parse(b"md ir"), Ok(Command::Mode(2)));
        assert_eq!(parse(b"md sp"), Ok(Command::Mode(3)));
        assert_eq!(parse(b"mode spread"), Ok(Command::Mode(3)));
        assert_eq!(parse(b"st"), Ok(Command::Status));
        assert_eq!(parse(b"he"), Ok(Command::Help));
        assert_eq!(parse(b"br 41"), Err(CommandError::Range));
        assert_eq!(parse(b"md xx"), Err(CommandError::Arguments));
        assert_eq!(parse(b"st 1"), Err(CommandError::Arguments));
    }

    #[test]
    fn overflow_discards_until_delimiter_and_disconnect_resets_partial_line() {
        let mut d = Decoder::new();
        for _ in 0..MAX_LINE + 1 {
            assert_eq!(d.push(b'x'), None);
        }
        for b in b"status" {
            assert_eq!(d.push(*b), None);
        }
        assert_eq!(d.push(b'\n'), Some(Err(CommandError::TooLong)));
        for b in b"mode uv" {
            d.push(*b);
        }
        d.reset();
        for b in b"status" {
            d.push(*b);
        }
        assert_eq!(d.push(b'\n'), Some(Ok(Command::Status)));
    }
}
