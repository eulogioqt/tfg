import requests
import base64
import uuid

BASE_URL = "http://localhost:7654/api/v1/faceprints"
HEADERS = {"Accept": "application/json", "Content-Type": "application/json"}
DUMMY_IMAGE = base64.b64encode(b"fake_image_bytes").decode("utf-8")

# Test de integración
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


def test_crud_faceprint():
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


def test_get_nonexistent_faceprint():
    non_existing_id = str(uuid.uuid4())
    response = requests.get(f"{BASE_URL}/{non_existing_id}", headers=HEADERS)
    assert response.status_code == 404
    assert "no encontrado" in response.text.lower()


def test_update_nonexistent_faceprint():
    non_existing_id = str(uuid.uuid4())
    response = requests.put(
        f"{BASE_URL}/{non_existing_id}",
        json={"name": "NuevoNombre"},
        headers=HEADERS,
    )
    assert response.status_code == 400
    assert "no existe ningun rostro" in response.text.lower()


def test_update_faceprint_without_name():
    create_resp = requests.post(BASE_URL, json={"name": "Temp", "image": DUMMY_IMAGE}, headers=HEADERS)
    created = create_resp.json()
    update_resp = requests.put(f"{BASE_URL}/{created['id']}", json={}, headers=HEADERS)
    assert update_resp.status_code == 400
    assert "no has incluido el campo name" in update_resp.text.lower()
    requests.delete(f"{BASE_URL}/{created['id']}", headers=HEADERS)


def test_delete_nonexistent_faceprint():
    non_existing_id = str(uuid.uuid4())
    response = requests.delete(f"{BASE_URL}/{non_existing_id}", headers=HEADERS)
    assert response.status_code == 400
    assert "no existe ningun rostro" in response.text.lower()


def test_create_faceprint_with_multiple_faces():
    # Suponiendo que el backend detectará múltiples rostros en esta imagen falsa
    image_data = base64.b64encode(b"fake_image_with_two_faces").decode("utf-8")
    response = requests.post(BASE_URL, json={"name": "TwoFaces", "image": image_data}, headers=HEADERS)
    assert response.status_code == 400
    assert "más de un rostro" in response.text.lower()


def test_create_faceprint_with_no_face():
    image_data = base64.b64encode(b"image_with_no_faces").decode("utf-8")
    response = requests.post(BASE_URL, json={"name": "NoFace", "image": image_data}, headers=HEADERS)
    assert response.status_code == 400
    assert "no se ha detectado ningún rostro" in response.text.lower()


def test_create_faceprint_with_low_score():
    image_data = base64.b64encode(b"face_with_low_score").decode("utf-8")
    response = requests.post(BASE_URL, json={"name": "LowScore", "image": image_data}, headers=HEADERS)
    assert response.status_code == 400
    assert "demasiado baja" in response.text.lower()


def test_create_faceprint_already_known():
    image_data = base64.b64encode(b"image_of_known_person").decode("utf-8")
    response = requests.post(BASE_URL, json={"name": "KnownUser", "image": image_data}, headers=HEADERS)
    assert response.status_code == 400
    assert "ya te conozco" in response.text.lower()
