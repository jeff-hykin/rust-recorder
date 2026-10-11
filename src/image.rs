//! Images: re-encoding raw `sensor_msgs.Image` frames, either for a ROS 2 `CompressedImage` (keeping full precision or
//! leaving the frame raw) or as dimos' `jpeg` storage codec (an LCM Image whose data is a JPEG, what JpegCodec reads).

use std::borrow::Cow;

use anyhow::{anyhow, bail, ensure, Result};
use jpeg_encoder::{ColorType, Encoder, SamplingFactor};
use serde::{Deserialize, Serialize};

use crate::lcm::{self, Value};

/// How a ROS 2 recording stores raw image frames. `Raw` keeps `sensor_msgs/Image`; the rest re-encode to
/// `sensor_msgs/CompressedImage`, unless the format would lose precision (16-bit depth in JPEG/WebP), when the frame
/// stays raw.
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug, Default)]
#[serde(rename_all = "lowercase")]
pub enum ImageFormat {
    #[default]
    Raw,
    Jpeg,
    Png,
    Webp,
    Jpegxl,
}

impl ImageFormat {
    /// The `format` a CompressedImage reader keys off.
    pub fn label(self) -> &'static str {
        match self {
            ImageFormat::Raw => "raw",
            ImageFormat::Jpeg => "jpeg",
            ImageFormat::Png => "png",
            ImageFormat::Webp => "webp",
            ImageFormat::Jpegxl => "jxl",
        }
    }
}

/// The fields of a decoded LCM `sensor_msgs.Image`, by name.
pub struct ImageFields {
    pub header: usize,
    pub height: usize,
    pub width: usize,
    pub encoding: usize,
    pub is_bigendian: usize,
    pub step: usize,
    pub data: usize,
}

pub fn image_fields() -> &'static ImageFields {
    static FIELDS: std::sync::OnceLock<ImageFields> = std::sync::OnceLock::new();
    FIELDS.get_or_init(|| {
        let image = lcm::registry().get("sensor_msgs.Image").expect("sensor_msgs.Image is vendored");
        let index = |name: &str| image.member_index(name).unwrap_or_else(|| panic!("sensor_msgs.Image.{name}"));
        ImageFields {
            header: index("header"),
            height: index("height"),
            width: index("width"),
            encoding: index("encoding"),
            is_bigendian: index("is_bigendian"),
            step: index("step"),
            data: index("data"),
        }
    })
}

/// A borrowed view of a decoded LCM Image.
pub struct RawImage<'a> {
    pub height: usize,
    pub width: usize,
    pub encoding: &'a str,
    pub big_endian: bool,
    pub step: usize,
    pub data: &'a [u8],
}

impl<'a> RawImage<'a> {
    pub fn from_value(value: &'a Value) -> Result<RawImage<'a>> {
        let fields = value.fields().ok_or_else(|| anyhow!("Image is not a struct"))?;
        let names = image_fields();
        let size = |index: usize| -> Result<usize> {
            let number = fields[index].as_i64().ok_or_else(|| anyhow!("Image dimension is not an integer"))?;
            usize::try_from(number).map_err(|_| anyhow!("negative Image dimension {number}"))
        };
        let Value::Str(encoding) = &fields[names.encoding] else { bail!("Image.encoding is not a string") };
        let Value::Bytes(data) = &fields[names.data] else { bail!("Image.data is not bytes") };
        Ok(RawImage {
            height: size(names.height)?,
            width: size(names.width)?,
            encoding: std::str::from_utf8(encoding).map_err(|_| anyhow!("Image.encoding is not UTF-8"))?,
            big_endian: fields[names.is_bigendian].as_i64().unwrap_or(0) != 0,
            step: size(names.step)?,
            data,
        })
    }

    /// dimos' JpegLcmTransport sends a plain Image whose data is a whole JPEG/PNG stream with step 0; a real raw frame
    /// fills exactly height*step bytes, which a codec stream essentially never does.
    pub fn container_format(&self) -> Option<&'static str> {
        if !matches!(self.encoding, "jpeg" | "jpg" | "png") || (self.step != 0 && self.height.checked_mul(self.step) == Some(self.data.len())) {
            return None;
        }
        if self.data.starts_with(&[0xFF, 0xD8]) {
            Some("jpeg")
        } else if self.data.starts_with(&[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]) {
            Some("png")
        } else {
            None
        }
    }
}

/// Pixels repacked tight (no row padding) in the channel order the encoders take.
enum Surface {
    Gray8(Vec<u8>),
    Rgb8(Vec<u8>),
    Rgba8(Vec<u8>),
    Gray16(Vec<u16>),
}

fn surface(image: &RawImage) -> Option<Surface> {
    let bytes_per_pixel = match image.encoding {
        "mono8" | "8UC1" => 1,
        "rgb8" | "8UC3" | "bgr8" => 3,
        "rgba8" | "8UC4" | "bgra8" => 4,
        "mono16" | "16UC1" => 2,
        _ => return None,
    };
    let tight = image.width.checked_mul(bytes_per_pixel)?;
    let step = if image.step > 0 { image.step } else { tight };
    if image.width == 0 || image.height == 0 || step < tight || image.data.len() < step.checked_mul(image.height)? {
        return None;
    }
    let rows = (0..image.height).map(|row| &image.data[row * step..row * step + tight]);
    if bytes_per_pixel == 2 {
        let big_endian = image.big_endian;
        return Some(Surface::Gray16(
            rows.flat_map(|row| row.chunks_exact(2))
                .map(|pair| if big_endian { u16::from_be_bytes([pair[0], pair[1]]) } else { u16::from_le_bytes([pair[0], pair[1]]) })
                .collect(),
        ));
    }
    let mut packed: Vec<u8> = rows.flatten().copied().collect();
    if image.encoding.starts_with("bgr") {
        for pixel in packed.chunks_exact_mut(bytes_per_pixel) {
            pixel.swap(0, 2);
        }
    }
    Some(match bytes_per_pixel {
        1 => Surface::Gray8(packed),
        3 => Surface::Rgb8(packed),
        _ => Surface::Rgba8(packed),
    })
}

/// Re-encodes a raw frame for a ROS 2 CompressedImage. None: the format cannot hold these pixels without losing
/// precision (or isn't built in), so the caller keeps the raw frame.
pub fn compress(image: &RawImage, format: ImageFormat, jpeg_quality: u8) -> Option<Vec<u8>> {
    let surface = surface(image)?;
    let (width, height) = (u32::try_from(image.width).ok()?, u32::try_from(image.height).ok()?);
    match format {
        ImageFormat::Raw => None,
        ImageFormat::Jpeg => to_jpeg(&surface, width, height, jpeg_quality, SamplingFactor::R_4_2_0),
        #[cfg(feature = "png")]
        ImageFormat::Png => to_png(&surface, width, height),
        #[cfg(feature = "webp")]
        ImageFormat::Webp => to_webp(&surface, width, height),
        #[cfg(feature = "jpegxl")]
        ImageFormat::Jpegxl => to_jpegxl(&surface, width, height),
        #[allow(unreachable_patterns)]
        _ => None,
    }
}

/// Whether a format is compiled in (png, webp and jpegxl are cargo features).
pub fn format_available(format: ImageFormat) -> bool {
    match format {
        ImageFormat::Raw | ImageFormat::Jpeg => true,
        ImageFormat::Png => cfg!(feature = "png"),
        ImageFormat::Webp => cfg!(feature = "webp"),
        ImageFormat::Jpegxl => cfg!(feature = "jpegxl"),
    }
}

/// Whether dimos' jpeg storage codec can take this frame (what the Rust recorder in dimos accepts).
pub fn dimos_jpeg_supports(encoding: &str) -> bool {
    matches!(encoding, "jpeg" | "rgb8" | "bgr8" | "rgba8" | "bgra8" | "mono8" | "mono16" | "16UC1" | "16SC1")
}

/// dimos' `jpeg` storage codec: the same LCM Image with encoding "jpeg", step 0 and JPEG data (quality and 4:2:2
/// subsampling as dimos' recorder). 16-bit frames keep their high byte, as dimos' Rust recorder does.
pub fn dimos_jpeg(image_value: &mut Value, quality: u8) -> Result<()> {
    let names = image_fields();
    let jpeg = {
        let image = RawImage::from_value(image_value)?;
        if image.encoding == "jpeg" {
            return Ok(());
        }
        ensure!(dimos_jpeg_supports(image.encoding), "the jpeg codec does not support image encoding {:?}", image.encoding);
        let surface = if matches!(image.encoding, "mono16" | "16UC1" | "16SC1") {
            let tight = image.width * 2;
            let step = if image.step > 0 { image.step } else { tight };
            ensure!(step >= tight && image.data.len() >= step * image.height, "16-bit image is smaller than its dimensions");
            let high = usize::from(!image.big_endian);
            Surface::Gray8((0..image.height).flat_map(|row| image.data[row * step..row * step + tight].chunks_exact(2).map(move |pixel| pixel[high])).collect())
        } else {
            surface(&image).ok_or_else(|| anyhow!("{}x{} {} image is smaller than its dimensions", image.width, image.height, image.encoding))?
        };
        to_jpeg(&surface, image.width as u32, image.height as u32, quality, SamplingFactor::R_4_2_2).ok_or_else(|| anyhow!("JPEG encoding failed"))?
    };
    let fields = image_value.fields_mut().unwrap();
    fields[names.encoding] = Value::Str(b"jpeg".to_vec());
    fields[names.is_bigendian] = Value::Int(0);
    fields[names.step] = Value::Int(0);
    fields[names.data] = Value::Bytes(jpeg);
    Ok(())
}

fn to_jpeg(surface: &Surface, width: u32, height: u32, quality: u8, sampling: SamplingFactor) -> Option<Vec<u8>> {
    let (bytes, color) = match surface {
        Surface::Gray8(bytes) => (bytes.as_slice(), ColorType::Luma),
        Surface::Rgb8(bytes) => (bytes.as_slice(), ColorType::Rgb),
        Surface::Rgba8(bytes) => (bytes.as_slice(), ColorType::Rgba),
        Surface::Gray16(_) => return None,
    };
    let mut out = Vec::new();
    let mut encoder = Encoder::new(&mut out, quality.clamp(1, 100));
    if !matches!(color, ColorType::Luma) {
        encoder.set_sampling_factor(sampling);
    }
    encoder.encode(bytes, u16::try_from(width).ok()?, u16::try_from(height).ok()?, color).ok()?;
    Some(out)
}

#[cfg(feature = "png")]
fn to_png(surface: &Surface, width: u32, height: u32) -> Option<Vec<u8>> {
    let (color, depth, bytes) = match surface {
        Surface::Gray8(bytes) => (png::ColorType::Grayscale, png::BitDepth::Eight, Cow::from(bytes)),
        Surface::Rgb8(bytes) => (png::ColorType::Rgb, png::BitDepth::Eight, Cow::from(bytes)),
        Surface::Rgba8(bytes) => (png::ColorType::Rgba, png::BitDepth::Eight, Cow::from(bytes)),
        // png stores 16-bit samples big-endian
        Surface::Gray16(values) => (png::ColorType::Grayscale, png::BitDepth::Sixteen, Cow::from(values.iter().flat_map(|value| value.to_be_bytes()).collect::<Vec<u8>>())),
    };
    let mut out = Vec::new();
    let mut encoder = png::Encoder::new(&mut out, width, height);
    encoder.set_color(color);
    encoder.set_depth(depth);
    let mut writer = encoder.write_header().ok()?;
    writer.write_image_data(&bytes).ok()?;
    writer.finish().ok()?;
    Some(out)
}

#[cfg(feature = "webp")]
fn to_webp(surface: &Surface, width: u32, height: u32) -> Option<Vec<u8>> {
    let (bytes, color) = match surface {
        Surface::Gray8(bytes) => (bytes.as_slice(), image_webp::ColorType::L8),
        Surface::Rgb8(bytes) => (bytes.as_slice(), image_webp::ColorType::Rgb8),
        Surface::Rgba8(bytes) => (bytes.as_slice(), image_webp::ColorType::Rgba8),
        Surface::Gray16(_) => return None,
    };
    let mut out = Vec::new();
    image_webp::WebPEncoder::new(&mut out).encode(bytes, width, height, color).ok()?;
    Some(out)
}

#[cfg(feature = "jpegxl")]
fn to_jpegxl(surface: &Surface, width: u32, height: u32) -> Option<Vec<u8>> {
    use zune_core::bit_depth::BitDepth;
    use zune_core::colorspace::ColorSpace;
    let (bytes, colorspace, depth) = match surface {
        Surface::Gray8(bytes) => (Cow::from(bytes), ColorSpace::Luma, BitDepth::Eight),
        Surface::Rgb8(bytes) => (Cow::from(bytes), ColorSpace::RGB, BitDepth::Eight),
        Surface::Rgba8(bytes) => (Cow::from(bytes), ColorSpace::RGBA, BitDepth::Eight),
        // zune reads 16-bit samples as native-endian byte pairs
        Surface::Gray16(values) => (Cow::from(values.iter().flat_map(|value| value.to_ne_bytes()).collect::<Vec<u8>>()), ColorSpace::Luma, BitDepth::Sixteen),
    };
    let options = zune_core::options::EncoderOptions::new(width as usize, height as usize, colorspace, depth);
    let mut out = Vec::new();
    zune_jpegxl::JxlSimpleEncoder::new(&bytes, options).encode(&mut out).ok()?;
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image(encoding: &str, width: usize, height: usize, step: usize, data: Vec<u8>) -> Value {
        let mut fields = vec![Value::Int(0); 8];
        let names = image_fields();
        fields[names.header] = Value::Struct(vec![Value::Int(0), Value::Struct(vec![Value::Int(12), Value::Int(34)]), Value::Str(b"cam".to_vec())]);
        fields[names.height] = Value::Int(height as i64);
        fields[names.width] = Value::Int(width as i64);
        fields[names.encoding] = Value::Str(encoding.as_bytes().to_vec());
        fields[names.step] = Value::Int(step as i64);
        fields[names.data] = Value::Bytes(data);
        fields[0] = Value::Int(0);
        Value::Struct(fields)
    }

    #[test]
    fn depth_never_goes_through_a_lossy_format() {
        let depth = image("16UC1", 4, 2, 8, vec![7; 16]);
        let view = RawImage::from_value(&depth).unwrap();
        assert!(compress(&view, ImageFormat::Jpeg, 90).is_none());
        assert!(compress(&view, ImageFormat::Webp, 90).is_none());
        if cfg!(feature = "png") {
            assert!(compress(&view, ImageFormat::Png, 90).unwrap().starts_with(&[0x89, b'P']));
        }
    }

    #[test]
    fn dimos_jpeg_keeps_the_envelope_and_handles_row_padding() {
        let mut frame = image("bgr8", 2, 2, 8, vec![9; 16]);
        dimos_jpeg(&mut frame, 50).unwrap();
        let view = RawImage::from_value(&frame).unwrap();
        assert_eq!((view.encoding, view.step, view.width, view.height), ("jpeg", 0, 2, 2));
        assert_eq!(&view.data[..2], &[0xFF, 0xD8]);
        let encoded = lcm::registry().encode("sensor_msgs.Image", &frame).unwrap();
        assert!(lcm::registry().decode("sensor_msgs.Image", &encoded).is_ok());
        let mut float_depth = image("32FC1", 1, 1, 4, vec![0; 4]);
        assert!(dimos_jpeg(&mut float_depth, 50).is_err());
    }
}
