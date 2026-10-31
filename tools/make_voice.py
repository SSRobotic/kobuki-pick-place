"""Make robot/sounds/<event>.wav from robot/voice_lines.py with a Microsoft Thai neural voice.

Only needed when a sentence changes (the .wav files are in the repo, the robot plays them offline).
Needs internet, the edge-tts package and ffmpeg (not part of the pixi environment):
    py -m pip install edge-tts
    py tools\\make_voice.py                       (female voice)
    py tools\\make_voice.py th-TH-NiwatNeural     (male voice)
"""
import asyncio
import pathlib
import subprocess
import sys
import tempfile

import edge_tts

ROBOT = pathlib.Path(__file__).resolve().parent.parent / "robot"
sys.path.insert(0, str(ROBOT))
from voice_lines import LINES  # noqa: E402

VOICE = sys.argv[1] if len(sys.argv) > 1 else "th-TH-PremwadeeNeural"


async def main():
    out = ROBOT / "sounds"
    out.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        for event, text in LINES.items():
            mp3 = pathlib.Path(tmp, event + ".mp3")
            for attempt in range(4):  # the online voice service drops a request now and then
                try:
                    await edge_tts.Communicate(text, VOICE).save(str(mp3))
                    break
                except edge_tts.exceptions.NoAudioReceived:
                    if attempt == 3:
                        raise
                    await asyncio.sleep(2)
            # 16-bit mono WAV: Windows plays it with the built-in winsound, Ubuntu with aplay
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp3), "-ac", "1", "-ar", "22050",
                            str(out / (event + ".wav"))], check=True)
            print(f"{event}.wav  {text}")


asyncio.run(main())
