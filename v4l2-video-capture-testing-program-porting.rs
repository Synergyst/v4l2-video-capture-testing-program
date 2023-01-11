//#![allow(dead_code)]
#![cfg_attr(debug_assertions, allow(dead_code, unused_imports))]
use std::process::exit;
use std::os::raw::c_int;
use std::os::raw::c_void;

const STARTING_WIDTH: c_int = 1280;
const STARTING_HEIGHT: c_int = 720;
const SCALED_OUT_WIDTH: c_int = 640;
const SCALED_OUT_HEIGHT: c_int = 360;

static CROP_MATRIX: [[i32; 4]; 2] = [[11, 4, 4, 2], [1, 1, 1, 1]];
static CROP_MATRIX_ALT: [[i32; 4]; 2] = [[11, 4, 4, 2], [1, 1, 1, 1]];
static mut CROPPED_WIDTH_ALT: i32 = 0;
static mut CROPPED_HEIGHT_ALT: i32 = 0;
static mut CROPPED_WIDTH: i32 = 0;
static mut CROPPED_HEIGHT: i32 = 0;

const MAX_SOBEL: i32 = 4 * 255;
const SOBEL_X: [[i32; 3]; 3] = [[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]];
const SOBEL_Y: [[i32; 3]; 3] = [[-1, -2, -1], [0, 0, 0], [1, 2, 1]];

static mut OUTPUT_FRAME: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME2: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_RGB24: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE1: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE2: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE3: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE4: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE5: Vec<u8> = Vec::new();
static mut RED_VALS: Vec<u8> = Vec::new();
static mut GREEN_VALS: Vec<u8> = Vec::new();
static mut BLUE_VALS: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_SCALED_R: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_SCALED_G: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_SCALED_B: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_ALT: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME2_ALT: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME3_ALT: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE_ALT: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE1_ALT: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE2_ALT: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE3_ALT: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE4_ALT: Vec<u8> = Vec::new();
static mut OUTPUT_FRAME_GREYSCALE5_ALT: Vec<u8> = Vec::new();

fn yuyv_to_greyscale(yuyv: &[u8], grey: &mut [u8], width: i32, height: i32) {
    for y in 0..height {
        for x in 0..width {
            let index = y * width + x;
            let y_index = index * 2;
            grey[index as usize] = yuyv[y_index as usize];
        }
    }
}

fn main() {
    // TODO: Add test function to send YUYV frame data to ported functions from C++ version
}
