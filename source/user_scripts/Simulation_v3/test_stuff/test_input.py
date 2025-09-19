# file: check_input_pipeline.py
from pprint import pprint
from InputManager import InputManager
from InputServer import InputServer
from fastapi.testclient import TestClient

def main():
    # 1) bootstrap server + client
    im = InputManager()
    server = InputServer(im)
    client = TestClient(server.app)

    # 2) sanity: empty snapshot
    print("Initial snapshot:")
    pprint(im.snapshot())

    # 3) press W on device kbd1
    r = client.post("/press", json={"device": "kbd1", "key": "W"})
    print("\nPOST /press →", r.status_code, r.json())

    # 4) check snapshot (W should be present for kbd1)
    snap = im.snapshot()
    print("\nAfter pressing W:")
    pprint(snap)
    assert "kbd1" in snap["keys"] and "W" in snap["keys"]["kbd1"], "W should be pressed on kbd1"

    # 5) press A as well (multi-key test)
    r = client.post("/press", json={"device": "kbd1", "key": "A"})
    print("\nPOST /press →", r.status_code, r.json())

    snap = im.snapshot()
    print("\nAfter pressing A too (W + A should be down):")
    pprint(snap)
    assert "A" in snap["keys"]["kbd1"], "A should be pressed on kbd1"

    # 6) release W
    r = client.post("/release", json={"device": "kbd1", "key": "W"})
    print("\nPOST /release →", r.status_code, r.json())

    snap = im.snapshot()
    print("\nAfter releasing W (only A should remain):")
    pprint(snap)
    assert ("W" not in snap["keys"]["kbd1"]) and ("A" in snap["keys"]["kbd1"]), "W should be up; A still down"

    print("\n✅ All checks passed.")

if __name__ == "__main__":
    main()
