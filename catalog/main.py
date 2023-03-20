import logging
import time
import json

from apache_atlas.client.base_client import AtlasClient
from atlas import AtlasHelper
from collector import DataCollector
import sqlite3

helper = None
conn = None
LOG = logging.getLogger('AV Tech Data Catalogging')

def init_atlas():
    try :
        client = AtlasClient('http://3.237.183.27:21000', ('admin', 'admin'))
        global helper
        helper = AtlasHelper(client)
        helper.createTypes()
        LOG.info("Connected to Apache Atlas server")
    except Exception as e :
        LOG.error("Error while connecting to Atlas server, %s", e)
        raise e


def init_db():
    try:
        global conn
        conn = sqlite3.connect("guid_data.db")
        LOG.info("Connected to SQLite database")
        conn.execute('''CREATE TABLE IF NOT EXISTS ENTITY (GUID TEXT NOT NULL, NAME TEXT NOT NULL);''')
        LOG.info("Table created successfully")
    except Exception as e:
        LOG.error("Exception while connecting to database, %s", e)
        raise e


def add_to_database(guid, entity_name):
    try:
        conn.execute(f'INSERT INTO ENTITY VALUES ("{guid}", "{entity_name}");')
        conn.commit()
    except Exception as e:
        LOG.error("Exception while adding values to database, %s", e)


def clear_database():
    try:
        conn.execute('DELETE FROM ENTITY;')
        conn.commit()
    except Exception as e:
        LOG.error("Exception while clearing database, %s", e)


def delete_entities():
    try:
        cur = conn.execute('SELECT * FROM ENTITY;')
        guids = [guid for guid, name in cur.fetchall()]
        if len(guids) > 0:
            helper.deleteEntity(guids)
    except Exception as e:
        LOG.error("Exception while deleting value from database, %s", e)


def run():
    delete_entities()
    clear_database()
    entities_info = DataCollector().collect()
    json.dump(entities_info, open('out', 'w'))
    for entity in ["type", "domain", "node", "topic", "service", "action"]:
        for name, info in entities_info[entity].items():
            guid = helper.createEntity(info)
            add_to_database(guid, name)
            LOG.info("created {}: {}".format(entity, guid))


if __name__ == "__main__":
    init_atlas()
    init_db()
    while True:
        run()
        time.sleep(18000)
