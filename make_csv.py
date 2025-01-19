import pandas
from com_adjuster import *

db = pandas.read_csv("com.csv")

print(db.loc[0])