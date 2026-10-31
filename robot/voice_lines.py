"""What the robot says for each /mission/event (Thai). The sound files are robot/sounds/<event>.wav,
made from this table by tools/make_voice.py. Change a sentence here, then run that script again."""

LINES = {
    "ready": "หุ่นยนต์พร้อมแล้ว",
    "go_station1": "เริ่มภารกิจ กำลังไปสถานีที่หนึ่ง",
    "arrived_station1": "ถึงสถานีที่หนึ่งแล้ว",
    "found_cube": "เจอวัตถุแล้ว",
    "picked": "หยิบวัตถุสำเร็จ",
    "go_station2": "กำลังไปสถานีที่สอง",
    "arrived_station2": "ถึงสถานีที่สองแล้ว",
    "found_station": "เจอจุดวางแล้ว",
    "placed": "วางวัตถุเรียบร้อย",
    "go_home": "กำลังกลับจุดเริ่มต้น",
    "arrived_home": "กลับถึงจุดเริ่มต้นแล้ว",
    "finished_succeeded": "ภารกิจสำเร็จแล้ว",
    "finished_aborted": "ภารกิจหยุดแล้ว",
    "retry": "ขอลองใหม่อีกครั้ง",
}
