import os, json, time

BASE_DIR = os.path.dirname(__file__)
CMD_DIR  = os.path.join(BASE_DIR, "commands")
CMD_PATH = os.path.join(CMD_DIR, "command.json")
os.makedirs(CMD_DIR, exist_ok=True)

def send_tap(row, col, say=None):
    cmd = {
        "id": int(time.time() * 1000),
        "type": "tap",
        "row": row,
        "col": col
    }
    if say:
        cmd["say"] = say

    with open(CMD_PATH, "w", encoding="utf-8") as f:
        json.dump(cmd, f)

    print("sent:", cmd)

if __name__ == "__main__":
    send_tap(1, 1, "Center!")
