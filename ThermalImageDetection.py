#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 概要：サーモ画像を撮影し、2値化処理を行い、画像内の物体推定を行う
# 実行コマンド：python3 thermal_detect_final.py

# -tf start-
import time
from absl import app, flags, logging
from absl.flags import FLAGS
import cv2
import numpy as np
import tensorflow as tf
from yolov3_tf2.models import (
    YoloV3, YoloV3Tiny
)
from yolov3_tf2.dataset import transform_images, load_tfrecord_dataset
from yolov3_tf2.utils import draw_outputs
import os
import os.path
from natsort import natsorted
import glob
import threading
import sys
# -tf end-

# -pure thermal start-
import requests
import json
import struct
import bz2
import serial
import base64
import io
from PIL import Image
from uvctypes import *
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import platform
# -pure thermal end-

sensor_measure_time = 3 # センサの計測間隔(秒)
device_id = '01' # センサノードID

# サーバのIPアドレス (自宅)
save_server_url = 'http://172.16.105.167:20000/Users/Infolab/Desktop/sensordata01/'

# 2値化画像保存先
ostu_file_path = '/home/pi/yolov3-tf2-master/input_image/'
if not os.path.exists(ostu_file_path):
    os.mkdir(ostu_file_path)

# 推定済画像保存先
output_file_path = '/home/pi/yolov3-tf2-master/output_image/'
if not os.path.exists(output_file_path):
    os.mkdir(output_file_path)

# サーモ撮影秒数
photo_sec = 2

# 推定開始フラグ
detect_strat_frag = False

# flags.DEFINE_string('classes', './data/coco.names', 'path to classes file')
# flags.DEFINE_string('weights', '/home/pi/learnig_model/TF2/yolov3-tiny.tf','path to weights file')
# flags.DEFINE_boolean('tiny', True, 'yolov3 or yolov3-tiny')
# flags.DEFINE_integer('size', 416, 'resize images to')
# flags.DEFINE_string('image', '/home/pi/Desktop/get_thermal_image/', 'path to input image')
# flags.DEFINE_string('tfrecord', None, 'tfrecord instead of image')
# flags.DEFINE_string('output', './output.png', 'path to output image')
# flags.DEFINE_integer('num_classes', 3, 'number of classes in the model')

# TF2の設定(参照:https://github.com/zzh8829/yolov3-tf2)
flags_tiny = True
num_classes = 3
weights_path = '/home/pi/learnig_model/TF2/yolov3-tiny.tf'
classes_path = '/home/pi/yolov3-tf2-master/data/coco.names'
resize = 416


"""""""""""""""""""""""""""""""""""""""""""""
Lepton3.5からサーモ画像を取得する関数
"""""""""""""""""""""""""""""""""""""""""""""
BUF_SIZE = 2
q = Queue(BUF_SIZE)
Image.LOAD_TRUNCATED_IMAGES = True
def py_frame_callback(frame, userptr):
    array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
    data = np.frombuffer(
    array_pointer.contents, dtype=np.dtype(np.uint16)
    ).reshape(
    frame.contents.height, frame.contents.width
    )
    # no copy
    # data = np.fromiter(
    #   frame.contents.data, dtype=np.dtype(np.uint8), count=frame.contents.data_bytes
    # ).reshape(
    #   frame.contents.height, frame.contents.width, 2
    # ) # copy
    if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
        return
    if not q.full():
        q.put(data)

PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(py_frame_callback)

def raw_to_8bit(data):
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(data, 8, data)
    return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

# kelvinからcelsiusへ
def kelvin_to_celsius(val_k):
    celsius = (val_k - 27315) / 100.0
    return celsius

def get_thermal():
    global celsius_min, celsius_max, detect_strat_frag
    n = 0
    save_cnt = 0
    photo_sec1 = time.time()
    ctx = POINTER(uvc_context)()
    dev = POINTER(uvc_device)()
    devh = POINTER(uvc_device_handle)()
    ctrl = uvc_stream_ctrl()
    res = libuvc.uvc_init(byref(ctx), 0)
    if res < 0:
        print("uvc_init error")
        exit(1)
    try:
        res = libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0)
        if res < 0:
            print("uvc_find_device error")
            exit(1)
        try:
            res = libuvc.uvc_open(dev, byref(devh))
            if res < 0:
                print("uvc_open error")
                exit(1)
            print("device opened!")
            print_device_info(devh)
            print_device_formats(devh)
            frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
            if len(frame_formats) == 0:
                print("device does not support Y16")
                exit(1)
            libuvc.uvc_get_stream_ctrl_format_size(devh, byref(ctrl), UVC_FRAME_FORMAT_Y16,
                frame_formats[0].wWidth, frame_formats[0].wHeight, int(1e7 / frame_formats[0].dwDefaultFrameInterval))
            res = libuvc.uvc_start_streaming(devh, byref(ctrl), PTR_PY_FRAME_CALLBACK, None, 0)
            if res < 0:
                print("uvc_start_streaming failed: {0}".format(res))
                exit(1)
            try:
                while True:
                    # 指定秒数経過したら撮影終了
                    photo_sec2 = time.time()
                    # print(photo_sec2 - photo_sec1)
                    if (photo_sec2 - photo_sec1) >= photo_sec:
                        print("指定秒数に達しました. 撮影を終了します")
                        break

                    data = q.get(True, 500)
                    if data is None:
                        break
                    data = cv2.resize(data[:,:], (640, 480))
                    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)
                    img = raw_to_8bit(data)
                    # celsius_min = kelvin_to_celsius(minVal) # サーモ画像内の最低温度
                    # celsius_max = kelvin_to_celsius(maxVal) # サーモ画像内の最高温度
                    # cv2.imwrite(os.path.join(img_file_path, 'thermal.png'), img)

                    # ndarrayのカラー画像をグレースケール化(参照:https://note.nkmk.me/python-numpy-rgb-image-split-color/)
                    img_gray = 0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]
                    pil_img_gray = np.uint8(img_gray)

                    # 大津の2値化
                    ret, img_otsu = cv2.threshold(pil_img_gray, 0, 255, cv2.THRESH_OTSU)
                    
                    # sikitiniyori nitika
                    # 二値化(閾値170~190を超えた画素を255にする)
                    threshold = 195
                    ret, img_thresh = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)

                    """
                    # 閾値がいくつになったか確認
                    print("閾値 : {}".format(ret))
                    # 画像の確認
                    cv2.imshow("otsu", img_otsu)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    """

                    # 2値化画像を保存
                    input = ostu_file_path + 'nichi_{}.png'.format(save_cnt)
                    save_cnt += 1
                    cv2.imwrite(input, img_thresh)

                    cv2.waitKey(1)
                cv2.destroyAllWindows()
            finally:
                libuvc.uvc_stop_streaming(devh)
            print("サーモカメラ撮影終了")
            # 推定開始フラグをTrue
            detect_strat_frag = True
        finally:
            libuvc.uvc_unref_device(dev)
    finally:
        libuvc.uvc_exit(ctx)


"""""""""""""""""""""""""""""""""""""""""""""
YOLOv3-Tinyの物体検出を行う
"""""""""""""""""""""""""""""""""""""""""""""
def main(_argv):
    global detect_strat_frag

    # 2値化画像保存先フォルダのファイルの削除
    if len(glob.glob(ostu_file_path + "*.png")) > 0:
        for p in glob.glob(ostu_file_path + "*.png", recursive=True):
            if os.path.isfile(p):
                os.remove(p)
        print("2値化画像を削除しました")
    else:
        print("2値化画像はありません")

    # 推定済画像保存先フォルダのファイルの削除
    if len(glob.glob(output_file_path + "*.png")) > 0:
        for p in glob.glob(output_file_path + "*.png", recursive=True):
            if os.path.isfile(p):
                os.remove(p)
        print("推定済画像を削除しました")
    else:
        print("推定済画像はありません")

    # プログラムの実行開始時間
    calculate_start_time = time.time()

    physical_devices = tf.config.experimental.list_physical_devices('GPU')
    if len(physical_devices) > 0:
        for device in physical_devices:
            tf.config.experimental.set_memory_growth(device, True)
            print('{} memory growth: {}'.format(device, tf.config.experimental.get_memory_growth(device)))
    else:
        print("Not enough GPU hardware devices available")

    logging.info('YoloV3Tiny loading...')
    
    if flags_tiny:
        yolo = YoloV3Tiny(classes=num_classes)
    else:
        yolo = YoloV3(classes=num_classes)
    
    logging.info('YoloV3Tiny loaded')

    calculate_finish_time = time.time()
    print("YoloV3Tinyロード時間 (開始 <--> YoloV3Tinyロード)：" + str(calculate_finish_time - calculate_start_time) + "秒" + "\n")

    yolo.load_weights(weights_path).expect_partial()
    logging.info('weights loaded')

    calculate_finish_time = time.time()
    print("学習モデルロード時間 (開始 <--> モデルロード)：" + str(calculate_finish_time - calculate_start_time) + "秒" + "\n")

    class_names = [c.strip() for c in open(classes_path).readlines()]
    logging.info('classes loaded')

    calculate_finish_time = time.time()
    print("クラスファイルロード時間 (開始 <--> ファイルロード)：" + str(calculate_finish_time - calculate_start_time) + "秒" + "\n")

    # 2値化画像保存先フォルダのファイルの数を読み込む
    dir_num = sum(os.path.isfile(os.path.join(ostu_file_path, name)) for name in os.listdir(ostu_file_path))

    # 2値化画像保存先フォルダのファイルのパスを読み込む
    image_path = glob.glob(ostu_file_path + "*.png")

    # 自然順ソート
    image = [None] * dir_num
    count = 0
    count_temp = 0
    detect_time_temp = 0
    detect_time = 0
    for path in natsorted(image_path):
        image[count] = path
        # print(image[count])
        count += 1
    count = 0

    # humanクラスの撮影回数
    human_frame_sum = 0

    # 推定の合計回数
    detect_count = 0

    # 推定結果保存用
    detect_class = [None] * dir_num * 2

    # 推定監視フラグがTrueになるまでループする
    while True:
        if detect_strat_frag == True:
            print("----------------推定開始----------------")
            break
    
    for i in range (dir_num):
        logging.info('input: {}'.format(image[i]))
        img_raw = tf.image.decode_image(
            open(image[i], 'rb').read(), channels=3)

        img = tf.expand_dims(img_raw, 0)
        img = transform_images(img, resize)

        # 画像1枚あたりの推定時間
        t1 = time.time()
        boxes, scores, classes, nums = yolo(img)
        t2 = time.time()
        logging.info('time: {}'.format(t2 - t1))
        detect_time_temp = t2 - t1
        detect_time = detect_time_temp + detect_time

        # 推定結果
        logging.info('detections:')
        for i in range(nums[0]):
            logging.info('\t{}, {}, {}'.format(class_names[int(classes[0][i])],
                                            np.array(scores[0][i]),
                                            np.array(boxes[0][i]))
                                            )
            # 多数決用の配列
            detect_class[count] = class_names[int(classes[0][i])]

            # 人間クラスと認識されたときに人間クラス撮影画像数を加算
            if 'human' in detect_class[count]:
                detect_count += 1
                human_temp = max(list(range(nums[0]))) + 1
                human_frame_sum = 1 / human_temp + human_frame_sum
            
            count += 1
        
        # 画像保存
        img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)
        img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
        output = '/home/pi/yolov3-tf2-master/output_image/' + 'detect_{}.png'.format(count_temp)
        count_temp += 1
        cv2.imwrite(output, img)
        logging.info('output saved to: {}'.format(output) + '\n')

    print("----------------推定終了----------------")

    print("画像1枚あたりに要した平均時間:" + str(detect_time / dir_num) + "秒" + "\n")
    
    # プログラムの実行終了時間
    calculate_finish_time = time.time()
    print("物体検出に要した合計時間：" + str(calculate_finish_time - calculate_start_time) + "秒" + "\n")
    print("推定した合計枚数 : " + str(dir_num) + "\n")

    # 検出されたクラスを数えるため
    human_front_num = 0
    human_right_num = 0
    human_left_num = 0
    none_class_num = 0
    detect_dict = {
        'human_front' : 0, 
        'human_right' : 0,
        'human_left' : 0,
        'none_class' : 0
    }

    # humanクラスの合計
    human_class_sum = 0

    # 検出されたクラスを数える
    for i in range (dir_num):
        if detect_class[i] == 'human_front':
            human_front_num += 1
            human_class_sum += 1
        elif detect_class[i] == 'human_right':
            human_right_num += 1
            human_class_sum += 1
        elif detect_class[i] == 'human_left':
            human_left_num += 1
            human_class_sum += 1
        else:
            none_class_num += 1
            
    detect_dict['human_front'] = human_front_num
    detect_dict['human_right'] = human_right_num
    detect_dict['human_left'] = human_left_num
    detect_dict['none_class'] = none_class_num
    print("推定された全てのクラス : " + str(detect_dict))
    
    # 上位3クラスを選択
    detect_res = []
    detect_res = sorted(detect_dict.items(), key=lambda x: x[1], reverse=True)[:3]
    print("上位3クラス : " + str(detect_res) + "タイプ: " + str(type(detect_res)))
    print("最上位クラス : " + str(detect_res[0][0]) + "タイプ: " + str(type(detect_res[0])))

    # 平均人数を計算
    if human_class_sum == 0 or human_frame_sum == 0:
        class_avg = 0
        # print("人間クラスは無い")
    else:
        class_avg = human_class_sum / human_frame_sum
    print("人間クラスと認識された画像内に存在する平均人数 : " + str((round((class_avg), 1))))

    # サーバへセンサデータを送信 (WiFi経由)
    send_data = {}
    send_data['id'] = device_id
    send_data['class'] = detect_res[0][0]
    send_data['avg'] = class_avg
    send_data['date'] = time.time() * 1000
    upload_func(send_data)


"""""""""""""""""""""""""""""""""""""""
センサーデータをサーバへ送信するテスト関数
"""""""""""""""""""""""""""""""""""""""
# 何回再接続要求するか (デフォルトは3回)
CONNECTION_RETRY = 3
# サーバのIPアドレス (自宅)
save_server_url = 'http://172.16.105.76:20000/Users/Infolab/Desktop/sensordata01/'

def upload_func(send_msg):
    print("send start!")
    print(send_msg)
    #sensor_data_sample = {'id': '01', 'class': 'human_front', 'avg_num': 2.2, 'date': 1609075340578.6162, 'location': {'lat': 34.99444966111906, 'lon': 135.95852828056923}}
    #sensor_data_sample = {'id': '01', 'class': 'human_front', 'avg_num': 2.2, 'date': time.time() * 1000}
    sensor_data_sample = send_msg
    send_files_json = json.dumps(sensor_data_sample) # 辞書 -> JSON
    send_sucess = False
    for i in range(1, CONNECTION_RETRY + 1):
        try:
            response = requests.post(save_server_url, json = send_files_json)
            print('サーバからのレスポンス : {0} {1}'.format(response.status_code, json.loads(response.text)["message"]))
        except Exception as e:
            print("サーバ送信エラー : " + str(e) + \
                " retry:{i}/{max}:wait{w}s"\
                .format(i = i , max = CONNECTION_RETRY, w = i * 5) + \
                "\n")
            time.sleep(i * 5)
        else:
            send_sucess = True
            break
    if send_sucess:
        print("送信成功！")
        pass
    else:
        print("送信失敗...データは破棄されます" + "\n")
    print("send end!")
    time.sleep(0.1)


if __name__ == '__main__':
    # time.sleep(5) # 野外撮影用
    thermal_thread = threading.Thread(target = get_thermal, args = ())
    thermal_thread.start()
    try:
        app.run(main)
    except SystemExit:
        pass
