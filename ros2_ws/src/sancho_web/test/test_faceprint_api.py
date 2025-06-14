import requests
import base64

BASE_URL = "http://localhost:7654/api/v1/faceprints"
HEADERS = {"Accept": "application/json", "Content-Type": "application/json"}
DUMMY_IMAGE = base64.b64encode(b"fake_image_bytes").decode("utf-8")


def test_create_and_get_faceprint():
    name = "TestUser1"
    payload = {"name": name, "image": DUMMY_IMAGE}

    create_resp = requests.post(BASE_URL, json=payload, headers=HEADERS)
    assert create_resp.status_code == 200
    created = create_resp.json()

    get_resp = requests.get(f"{BASE_URL}/{created['id']}", headers=HEADERS)
    assert get_resp.status_code == 200
    fetched = get_resp.json()

    assert fetched["name"] == name
    assert fetched["id"] == created["id"]

    requests.delete(f"{BASE_URL}/{created['id']}", headers=HEADERS)


def test_update_faceprint():
    create_resp = requests.post(BASE_URL, json={"name": "UserToUpdate", "image": DUMMY_IMAGE}, headers=HEADERS)
    created = create_resp.json()

    update_resp = requests.put(f"{BASE_URL}/{created['id']}", json={"name": "UpdatedUser"}, headers=HEADERS)
    assert update_resp.status_code == 200
    updated = update_resp.json()

    get_resp = requests.get(f"{BASE_URL}/{created['id']}", headers=HEADERS)
    fetched = get_resp.json()

    assert updated["name"] == "UpdatedUser"
    assert fetched["name"] == "UpdatedUser"

    requests.delete(f"{BASE_URL}/{created['id']}", headers=HEADERS)


def test_delete_faceprint():
    create_resp = requests.post(BASE_URL, json={"name": "UserToDelete", "image": DUMMY_IMAGE}, headers=HEADERS)
    created = create_resp.json()

    delete_resp = requests.delete(f"{BASE_URL}/{created['id']}", headers=HEADERS)
    assert delete_resp.status_code == 200
    assert "borrado correctamente" in delete_resp.json()["details"].lower()


def test_get_all_faceprints():
    names = ["User1", "User2", "User3"]
    ids = []
    for name in names:
        resp = requests.post(BASE_URL, json={"name": name, "image": DUMMY_IMAGE}, headers=HEADERS)
        ids.append(resp.json()["id"])

    get_all_resp = requests.get(BASE_URL, headers=HEADERS)
    assert get_all_resp.status_code == 200
    all_faceprints = get_all_resp.json()
    all_ids = {fp["id"] for fp in all_faceprints}

    for fid in ids:
        assert fid in all_ids

    for fid in ids:
        requests.delete(f"{BASE_URL}/{fid}", headers=HEADERS)


def test_get_single_faceprint():
    create_resp = requests.post(BASE_URL, json={"name": "SingleUser", "image": DUMMY_IMAGE}, headers=HEADERS)
    created = create_resp.json()

    get_resp = requests.get(f"{BASE_URL}/{created['id']}", headers=HEADERS)
    assert get_resp.status_code == 200
    fetched = get_resp.json()

    assert fetched["name"] == "SingleUser"
    assert fetched["id"] == created["id"]

    requests.delete(f"{BASE_URL}/{created['id']}", headers=HEADERS)


def test_end_to_end_crud_faceprint():
    payload = {"name": "E2EUser", "image": DUMMY_IMAGE}

    create_resp = requests.post(BASE_URL, json=payload, headers=HEADERS)
    created = create_resp.json()

    get_resp = requests.get(f"{BASE_URL}/{created['id']}", headers=HEADERS)
    fetched = get_resp.json()
    assert fetched["name"] == "E2EUser"

    update_resp = requests.put(f"{BASE_URL}/{created['id']}", json={"name": "FinalName"}, headers=HEADERS)
    updated = update_resp.json()
    assert updated["name"] == "FinalName"

    delete_resp = requests.delete(f"{BASE_URL}/{created['id']}", headers=HEADERS)
    result = delete_resp.json()
    assert "borrado correctamente" in result["details"].lower()
