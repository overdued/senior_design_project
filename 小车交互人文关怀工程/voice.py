from pydub import AudioSegment
from pydub.playback import play

audio = AudioSegment.from_file("Michael Hunter.ogg", format = "ogg")
adjusted_audio = audio.apply_gain(-20)
play(adjusted_audio)