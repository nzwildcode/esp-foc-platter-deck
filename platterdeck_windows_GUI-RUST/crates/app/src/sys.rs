//! Windows console plumbing. The release build is a GUI-subsystem binary
//! with no console of its own, so `--list`, `--selftest` and the log lines
//! would vanish when run from a shell. Borrow the launching shell's console
//! instead.

/// Attach to the parent console and wire stdout/stderr to it when the
/// process started without usable standard handles. No-op when stdout is
/// already a console, a pipe or a file (debug build, or redirected), and
/// when there is no parent console at all (double-clicked).
#[cfg(windows)]
pub fn attach_parent_console() {
    use std::os::windows::io::AsRawHandle;
    use windows_sys::Win32::Foundation::{HANDLE, INVALID_HANDLE_VALUE};
    use windows_sys::Win32::System::Console::{
        AttachConsole, GetStdHandle, SetStdHandle, ATTACH_PARENT_PROCESS, STD_ERROR_HANDLE, STD_OUTPUT_HANDLE,
    };

    fn usable(h: HANDLE) -> bool {
        !h.is_null() && h != INVALID_HANDLE_VALUE
    }

    unsafe {
        if usable(GetStdHandle(STD_OUTPUT_HANDLE)) && usable(GetStdHandle(STD_ERROR_HANDLE)) {
            return;
        }
        if AttachConsole(ATTACH_PARENT_PROCESS) == 0 {
            return;
        }
        for id in [STD_OUTPUT_HANDLE, STD_ERROR_HANDLE] {
            if usable(GetStdHandle(id)) {
                continue;
            }
            // Rust's std looks the handle up on every write, so setting it
            // here makes println!/eprintln!/env_logger land on the console.
            if let Ok(f) = std::fs::OpenOptions::new().write(true).open("CONOUT$") {
                SetStdHandle(id, f.as_raw_handle() as HANDLE);
                std::mem::forget(f);
            }
        }
    }
}

#[cfg(not(windows))]
pub fn attach_parent_console() {}

/// Native Unicode Windows audio file picker, owned by the deck window.
#[cfg(windows)]
pub fn choose_track() -> Option<std::path::PathBuf> {
    use std::os::windows::ffi::OsStringExt;
    use windows_sys::Win32::UI::Controls::Dialogs::*;
    use windows_sys::Win32::UI::WindowsAndMessaging::GetForegroundWindow;
    let mut file = [0u16; 32768];
    let filter: Vec<u16> = "Audio files\0*.mp3;*.wav;*.flac;*.m4a;*.aac;*.ogg;*.opus;*.mp4\0All files\0*.*\0\0".encode_utf16().collect();
    let title: Vec<u16> = "Choose a track\0".encode_utf16().collect();
    unsafe {
        let mut dialog: OPENFILENAMEW = std::mem::zeroed();
        dialog.lStructSize = std::mem::size_of::<OPENFILENAMEW>() as u32;
        dialog.hwndOwner = GetForegroundWindow();
        dialog.lpstrFilter = filter.as_ptr();
        dialog.lpstrFile = file.as_mut_ptr();
        dialog.nMaxFile = file.len() as u32;
        dialog.lpstrTitle = title.as_ptr();
        dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | OFN_EXPLORER;
        if GetOpenFileNameW(&mut dialog) == 0 { return None; }
    }
    let n = file.iter().position(|&c| c == 0).unwrap_or(file.len());
    Some(std::ffi::OsString::from_wide(&file[..n]).into())
}

#[cfg(not(windows))]
pub fn choose_track() -> Option<std::path::PathBuf> { None }
