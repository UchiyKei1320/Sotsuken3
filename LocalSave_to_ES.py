#!/usr/bin/env python3
from flask import Flask, request, Response, make_response, jsonify
import os
import werkzeug
from datetime import datetime
import bz2
import json
import base64
from io import BytesIO
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import cv2
from elasticsearch import Elasticsearch
import smtplib
from email.mime.text import MIMEText

smtp_host = 'smtp.office365.com'
smtp_port = 587
from_email = 'uchiyama_keigo1320@outlook.jp'
to_email = 'keigo_stranger@ezweb.ne.jp'
username = 'uchiyama_keigo1320@outlook.jp'
password = 'keigo1005'

app = Flask(__name__, static_folder=None)
savedir_path = "/Users/Infolab/Desktop/sensordata01"

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
thermal_detect_final.pyから送信されたメッセージを受信するプログラム
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

@app.route('/Users/Infolab/Desktop/sensordata01/', methods=['POST'])
def upload_log_sensor01():
    json_data = request.get_json() # POSTされたjsonを取得
    dict_data = json.loads(json_data) # jsonを辞書に変換
    print("紐付け前 : " + str(dict_data))

    "---画像データの処理---"
    #img = dict_data["img"] # base64を取り出す # str
    #image_dec = base64.b64decode(img) # base64に変換された画像データを元のバイナリデータに変換 # bytes
    #img = BytesIO(img) # _io.BytesIO pillowで扱えるように変換
    #img = Image.open(img)
    #img_shape = img.size # 取得した画像で適当に処理
    #data_np = np.fromstring(image_dec, dtype='uint8')
    #decimg = cv2.imdecode(data_np, 1)

    "---センサデータの処理---"
    #sensor_data = dict_data["sensor_data"]
    #print("sensor_data : " + str(sensor_data)) # DEBUG:
    #print("sensor_data(type) : " + str(type(sensor_data))) # DEBUG: <class 'dict'>
    #sensor_data_file = str(sensor_data) # 辞書型から文字列へ変換
    #print("sensor_data_file : " + str(sensor_data_file)) # DEBUG:
    #print("sensor_data_file(type) : " + str(type(sensor_data_file))) # DEBUG: <class 'str'>

    "---解析データの処理---"
    sensor_data = dict_data
    # print("sensor_data : " + str(sensor_data)) # DEBUG:
    # print("sensor_data(type) : " + str(type(sensor_data))) # DEBUG: <class 'dict'>
    # print("sensor_id : " + str(sensor_data['id']))
    sensor_id = sensor_data['id']
    # センサIDと位置情報を組み合わせる
    if sensor_id == "01":
        sensor_data['location'] = {'lat': 34.99444966111906, 'lon': 135.95852828056923} # 自宅
        print("紐付け後 : " + str(sensor_data))
    sensor_data_file = str(sensor_data) # 辞書型から文字列へ変換
    # print("sensor_data_file : " + str(sensor_data_file)) # DEBUG:
    # print("sensor_data_file(type) : " + str(type(sensor_data_file))) # DEBUG: <class 'str'>

    "---メール通知---"
    body = "草津市野路町にて、害獣を検知しました。" # 本文
    msg = MIMEText(body)
    msg['Subject'] = '【害獣検知】' # タイトル
    msg['From'] = from_email # 送信元
    msg['To'] = to_email # 送信先
    server = smtplib.SMTP(smtp_host, smtp_port)
    server.ehlo()
    server.starttls()
    server.ehlo()
    server.login(username, password)
    server.send_message(msg)
    server.quit()

    "---保存先フォルダの処理---"
    # 西暦と月のフォルダを作成
    nowtime = datetime.now()
    savedir = savedir_path
    if not os.path.exists(os.path.join(savedir)):
        os.mkdir(os.path.join(savedir))
    # print("MAKE_DIR: " + savedir) # DEBUG:

    # 年のフォルダを作成
    savedir += datetime.now().strftime("/%Y")
    if not os.path.exists(os.path.join(savedir)):
        os.mkdir(os.path.join(savedir))
    # print("MAKE_DIR: " + savedir) # DEBUG:

    # 月のフォルダ作成
    savedir += nowtime.strftime("/%m")
    if not os.path.exists(os.path.join(savedir)):
        os.mkdir(os.path.join(savedir))
    # print("MAKE_DIR: " + savedir) # DEBUG:

    # 日のフォルダを生成
    savedir += nowtime.strftime("/%d")
    if not os.path.exists(os.path.join(savedir)):
        os.mkdir(os.path.join(savedir))
    # print("MAKE_DIR: " + savedir) # DEBUG:

    "---保存先パスの作成---"
    savefile = savedir
    # print("savefile : " + str(savefile)) # DEBUG:
    savefile_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    # print("savefile_name : " + str(savefile_name)) # DEBUG:
    sensor_savefile_path = savefile + "/" + savefile_name + ".json"
    # print("sensor_savefile_path : " + str(sensor_savefile_path)) # DEBUG:

    "---画像データ保存---"
    # img_savefile_path = savefile + "/" + savefile_name + ".png"
    # cv2.imwrite(img_savefile_path, decimg)

    "---センサデータ保存---"
    # with open(sensor_savefile_path, mode='w') as f:
        # f.write(sensor_data_file)
    # print(str(savedir) + "に保存しました")

    "---解析データ保存---"
    with open(sensor_savefile_path, mode='w') as f:
        f.write(sensor_data_file)
    print(str(savedir) + "に保存しました")

    # Elasticsearchにデータを投入
    es = Elasticsearch(host = 'localhost', port = '9200')
    print(es.indices.exists(index = 'my_index'))
    res = es.index(index = 'my_index', doc_type = 'my_type', body = dict_data)
    print(res)

    # HTTPレスポンスを送信
    return Response(
        response = json.dumps(
            {
                "message": "upload success!"
            }),
        status = 200)
    # HTTPレスポンスを送信(保存したファイルパス名)
    """
    return Response(
        response = json.dumps(
            {
                "message": "{} is \"sensor_data\" save path".format(sensor_savefile_path) + "\n"
                            "{} is \"sensor_data\" save path".format(filename)
            }),
        status = 200)
    """


if __name__ == "__main__":
    print(app.url_map)
    app.run(host = '0.0.0.0', port = 20000, threaded = True)
