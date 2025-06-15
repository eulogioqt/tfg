from hri_vision.database.faceprints_database import FaceprintsDatabase

# Black-box: verify add and get operations work without saving

def test_add_and_get_faceprint():
    db = FaceprintsDatabase(db_path="unused.json", db_mode="no_save")
    entry = db.add("Alice", [1.0], "img", 0.9)
    assert db.get_by_id(entry["id"]) == entry
    assert entry["name"] == "Alice"


# White-box: update modifies stored data

def test_update_faceprint():
    db = FaceprintsDatabase(db_path="unused.json", db_mode="no_save")
    entry = db.add("Bob", [1.0], "img", 0.9)
    db.update(entry["id"], {"name": "Robert"})
    assert db.get_by_id(entry["id"])["name"] == "Robert"


# White-box: removing deletes entry

def test_remove_faceprint():
    db = FaceprintsDatabase(db_path="unused.json", db_mode="no_save")
    entry = db.add("Carol", [1.0], "img", 0.9)
    db.remove(entry["id"])
    assert db.get_by_id(entry["id"]) is None


# Hybrid: filter get_all by name

def test_get_all_by_name():
    db = FaceprintsDatabase(db_path="unused.json", db_mode="no_save")
    db.add("Dave", [1.0], "img", 0.9)
    db.add("Eve", [2.0], "img", 0.8)
    results = db.get_all("Eve")
    assert len(results) == 1
    assert results[0]["name"] == "Eve"
