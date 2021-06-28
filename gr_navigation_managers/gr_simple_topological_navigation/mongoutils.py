import pymongo
import yaml
import json

class MongoManager(object):
    def __init__(self, dbname="execution_data"):
        self.client = pymongo.MongoClient(host="localhost", port=62345)
        if dbname in self.client.list_database_names():
            self.database = self.client.get_database(dbname)
        else:
            self.database = self.client[dbname]

    def insert_in_collection(self,rosmsg, collname):
        if collname in self.database.list_collections():
            collection = self.client.get_collection(collname)
        else:
            collection = self.database[collname]
        jsonmsg = json.dumps(yaml.load(str(rosmsg), Loader=yaml.FullLoader))
        jsonmsg2 = json.loads(jsonmsg)
        print "msgs in json ", jsonmsg2, jsonmsg
        collection.insert(jsonmsg2)
