# app.py (PC 측)
from flask import Flask

app = Flask(__name__)


@app.route('/')
def hello():
    return "hello pi Flask!"

if __name__ =="__main__":
    app.run(host="192.168.100.201", port=8080)


