import pymongo
import yaml
import json
import time

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class MongoManager(object):
    def __init__(self, dbname="execution_data"):
        self.client = pymongo.MongoClient(host="localhost", port=27017)
        if dbname in self.client.list_database_names():
            self.database = self.client.get_database(dbname)
        else:
            self.database = self.client[dbname]

    def get_collection(self, collname):
        if collname in self.database.list_collections():
            collection = self.client.get_collection(collname)
        else:
            collection = self.database[collname]
        return collection

    def insert_in_collection(self,rosmsg, collname):
        collection = self.get_collection(collname)
        jsonmsg = json.dumps(yaml.load(str(rosmsg), Loader=yaml.Loader))
        jsonmsg2 = json.loads(jsonmsg)
        collection.insert(jsonmsg2)

    def query(self,collname,query={}):
        exec_time = 0.0
        covered_distance = 0.0
        collection = self.get_collection(collname)
        for entry in collection.find(query):
            exec_time +=entry["time_of_execution"]
            covered_distance +=entry["covered_distance"]

        #print(exec_time, covered_distance)
        mystring =  "Execution Time {} seconds \n ".format(exec_time) + "\n Covered Distance {} \n".format(covered_distance)
        print(mystring)
        return mystring
        return bcolors.OKCYAN + "Execution Time {} seconds".format(exec_time) + bcolors.OKCYAN + "Covered Distance {} \n".format(covered_distance)

if __name__ == "__main__":
    mm = MongoManager()
    tasks = ["CUT", "COLLECT"]
    while True:
        with open('/var/www/www.grassrobotics/example.txt','r+') as myfile:
            data = myfile.read()
            myfile.seek(0)
            for t in tasks:
                #print (bcolors.HEADER + "TASK " , t)
                myfile.write("Stats during {} \n ".format(t))
                myfile.write("\n")
                #print (bcolors.OKBLUE + "WORKING Mode")
                myfile.write("Working Stats \n ")
                myfile.write(mm.query(t, {"action":"RUN"}))
                myfile.write("\n")
                #print (bcolors.OKBLUE + "Change row Mode")
                myfile.write("Stats during Change Row \n")
                myfile.write(mm.query(t, {"action":"CHANGE_ROW"}))
                myfile.write("\n")
                #print (bcolors.OKBLUE + "TOTAL")
                myfile.write("TOTAL stats for mode {} \n". format(t))
                myfile.write(mm.query(t))
                myfile.write("\n \n")
            myfile.truncate()
        time.sleep(30)
