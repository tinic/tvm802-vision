#pragma once
// Win32 settings UI entry point. Windows-only (the rest of the detector stays
// Windows-free); compiled into the DLL, not the off-target test harness.

namespace vis {

// Start the settings UI background thread (idempotent / no-op if already started):
// loads the persisted settings file, registers the global hotkey (Ctrl+Alt+M), and
// serves the modeless settings dialog. Called lazily from the shim on first frame so
// it never runs during DllMain (loader lock).
void start_settings_ui();

}  // namespace vis
