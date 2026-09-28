#!/usr/bin/env python3
"""
gen_voice_prompts.py — generate the full-sentence Vietnamese voice prompts the
firmware plays for "speed limit ahead" and "camera ahead" (ui/Dashboard.cpp).

One MP3 per complete sentence (number included), so a prompt is one natural
utterance in one voice instead of two clips stitched together:

  speed_next/<N>.mp3    "Giới hạn tốc độ tiếp theo là N ki-lô-mét trên giờ"
  camera_limit/<N>.mp3  "Phía trước có camera giám sát tốc độ, giới hạn N ki-lô-mét trên giờ"
  camera_ahead.mp3      "Phía trước có camera giám sát tốc độ"

Output format matches the device (audio/AudioPlayer.cpp: I2S at 16 kHz):
MP3, 16 kHz, mono. Copy the output into /speedmap/sounds/vi/ on the SD card.
The firmware falls back to the old two-clip prompts when these files are absent.

Needs: pip install edge-tts imageio-ffmpeg   (edge-tts uses Microsoft's online TTS)

Usage:
    python tools/gen_voice_prompts.py [out_dir] [--voice vi-VN-HoaiMyNeural|vi-VN-NamMinhNeural]
"""
import asyncio
import subprocess
import sys
import tempfile
from pathlib import Path

import edge_tts
import imageio_ffmpeg

SPEEDS = [20, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100, 110, 120]
UNIT = "ki-lô-mét trên giờ"


def prompts():
    for v in SPEEDS:
        yield f"speed_next/{v}.mp3", f"Giới hạn tốc độ tiếp theo là {v} {UNIT}."
        yield f"camera_limit/{v}.mp3", f"Phía trước có camera giám sát tốc độ, giới hạn {v} {UNIT}."
    yield "camera_ahead.mp3", "Phía trước có camera giám sát tốc độ."


async def synth(text, voice, dst):
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    with tempfile.TemporaryDirectory() as td:
        raw = Path(td) / "raw.mp3"
        await edge_tts.Communicate(text, voice, rate="+5%").save(str(raw))
        dst.parent.mkdir(parents=True, exist_ok=True)
        # 16 kHz mono MP3; trim leading/trailing silence so prompts start promptly.
        subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-i", str(raw),
                        "-af", "silenceremove=start_periods=1:start_threshold=-45dB,"
                               "areverse,silenceremove=start_periods=1:start_threshold=-45dB,areverse",
                        "-ar", "16000", "-ac", "1", "-b:a", "64k", str(dst)], check=True)


async def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    voice = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--voice=")), "vi-VN-HoaiMyNeural")
    out = Path(args[0]) if args else Path(__file__).resolve().parent.parent / "data" / "speedmap" / "sounds" / "vi"
    force = "--force" in sys.argv
    n = 0
    for rel, text in prompts():
        dst = out / rel
        if dst.exists() and dst.stat().st_size > 0 and not force:
            print(f"{rel:24s} (exists)")
            continue
        for attempt in range(5):  # the online TTS occasionally returns no audio when called in bursts
            try:
                await synth(text, voice, dst)
                break
            except edge_tts.exceptions.NoAudioReceived:
                await asyncio.sleep(2 + 3 * attempt)
        else:
            raise SystemExit(f"TTS failed for {rel}")
        n += 1
        print(f"{rel:24s} {text}")
    print(f"{n} prompts -> {out} (voice {voice})")


if __name__ == "__main__":
    asyncio.run(main())
