# server.py
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
import threading
import socket
from pytz import timezone

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

# Configure the database
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///sensor_data.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

# Define the database models
class SensorData(db.Model):
    def get_beijing_time(self, beijing_tz=None):
        return datetime.now(beijing_tz)  # 直接获取当前北京时间
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, default=get_beijing_time)
    device_type = db.Column(db.String(50))
    ip = db.Column(db.String(50))
    temperature = db.Column(db.String(10))
    humidity = db.Column(db.String(10))

class LightData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    device_type = db.Column(db.String(50))
    ip = db.Column(db.String(50))
    light_value = db.Column(db.Integer)  # 光照值

# Initialize the database
with app.app_context():
    db.create_all()

clients = {}  # {addr_str: conn}
client_types = {}  # {addr_str: sensor_type}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/realtime_data', methods=['GET'])
def get_realtime_data():
    data = SensorData.query.order_by(SensorData.timestamp.desc()).limit(10).all()
    return jsonify([{
        'timestamp': d.timestamp.strftime('%Y-%m-%d %H:%M:%S'),
        'device_type': d.device_type,
        'ip': d.ip,
        'temperature': d.temperature,
        'humidity': d.humidity
    } for d in data])

@app.route('/api/history_data', methods=['GET'])
def get_history_data():
    start_time = request.args.get('start_time')
    end_time = request.args.get('end_time')
    query = SensorData.query

    # 过滤时间范围
    if start_time:
        try:
            query = query.filter(SensorData.timestamp >= datetime.fromisoformat(start_time))
        except ValueError:
            return jsonify([])  # 如果时间格式错误，返回空数组
    if end_time:
        try:
            query = query.filter(SensorData.timestamp <= datetime.fromisoformat(end_time))
        except ValueError:
            return jsonify([])  # 如果时间格式错误，返回空数组

    # 查询结果
    data = query.order_by(SensorData.timestamp.desc()).all()
    return jsonify([{
        'timestamp': d.timestamp.strftime('%Y-%m-%d %H:%M:%S'),
        'device_type': d.device_type,
        'ip': d.ip,
        'temperature': d.temperature,
        'humidity': d.humidity
    } for d in data])

@app.route('/api/realtime_light', methods=['GET'])
def get_realtime_light():
    data = LightData.query.order_by(LightData.timestamp.desc()).limit(10).all()
    return jsonify([{
        'timestamp': d.timestamp.strftime('%Y-%m-%d %H:%M:%S'),
        'device_type': d.device_type,
        'ip': d.ip,
        'light_value': d.light_value
    } for d in data])

@app.route('/api/history_light', methods=['GET'])
def get_history_light():
    start_time = request.args.get('start_time')
    end_time = request.args.get('end_time')
    query = LightData.query

    # 过滤时间范围
    if start_time:
        try:
            query = query.filter(LightData.timestamp >= datetime.fromisoformat(start_time))
        except ValueError:
            return jsonify([])  # 如果时间格式错误，返回空数组
    if end_time:
        try:
            query = query.filter(LightData.timestamp <= datetime.fromisoformat(end_time))
        except ValueError:
            return jsonify([])  # 如果时间格式错误，返回空数组

    # 查询结果
    data = query.order_by(LightData.timestamp.desc()).all()
    return jsonify([{
        'timestamp': d.timestamp.strftime('%Y-%m-%d %H:%M:%S'),
        'device_type': d.device_type,
        'ip': d.ip,
        'light_value': d.light_value
    } for d in data])

@socketio.on('send_message')
def handle_send_message(data):
    msg = data['message']
    target = data.get('target')

    if not clients:
        emit('log', {'message': '无连接的 TCP 客户端'})
        return

    try:
        if target == 'all' or not target:
            for addr, conn in clients.items():
                conn.send(msg.encode('utf-8'))
            emit('log', {'message': f'你(广播): {msg}'})
        else:
            conn = clients.get(target)
            if conn:
                conn.send(msg.encode('utf-8'))
                emit('log', {'message': f'你({target}): {msg}'})
            else:
                emit('log', {'message': f'目标 {target} 未连接'})
    except Exception as e:
        emit('log', {'message': f'发送失败: {e}'})

def handle_client(conn, addr):
    addr_str = f"{addr[0]}:{addr[1]}"
    print(f"[TCP] 客户端已连接: {addr_str}")
    clients[addr_str] = conn
    socketio.emit('log', {'message': f'客户端已连接: {addr_str}'})
    socketio.emit('client_list', [{'addr': a, 'type': client_types.get(a, '未知')} for a in clients.keys()])

    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break

            print("接收到消息：", data.decode('utf-8'))
        
            # 去除结尾换行
            hex_str = data.strip().decode()

            # 每两个字符组成一个字节（十六进制）
            msg = [int(hex_str[i:i+2], 16) for i in range(0, len(hex_str), 2)]

            if len(msg) < 10:
                continue

            sensor_type = msg[7]
            data_len = msg[9]
            payload = msg[10:10 + data_len]
            readable = {}
            
            
            if sensor_type == 0xa0:
                sensor = '温湿度'
                temp = bytes(payload[:2]).decode(errors='ignore')
                humi = bytes(payload[2:4]).decode(errors='ignore')
                readable = {'temp': temp, 'humi': humi}
                # print("data: ", data)
                # print("msg: ", msg)
                # print("bottom:", temp, msg[12], msg[13])

                # Save to database
                with app.app_context():  # ✅ 添加应用上下文
                    new_data = SensorData(
                        device_type=sensor,
                        ip=addr_str,
                        temperature=temp,
                        humidity=humi
                    )
                    db.session.add(new_data)
                    db.session.commit()
            elif sensor_type == 0xa1:
                sensor = '风扇'
                readable = {'status': '已开启' if payload[0] == 1 else '关闭'}
            elif sensor_type == 0xa2:
                sensor = '蜂鸣器'
                readable = {'status': '已开启' if payload[0] == 1 else '关闭'}
            elif sensor_type == 0xa3:  # 光照传感器
                sensor = '光照'
                # light_value = int(''.join(f"{byte:02x}" for byte in payload[:4]))  # 解析光照值
                # light_value = int(light_value)  # 去掉前导零
                light_value = int(''.join(chr(b) for b in payload[:4]).lstrip('0') or '0')
                readable = {'light': light_value}

                # Save to database
                with app.app_context():
                    new_data = LightData(
                        device_type=sensor,
                        ip=addr_str,
                        light_value=light_value
                    )
                    db.session.add(new_data)
                    db.session.commit()
            elif sensor_type == 0xa4:
                sensor = '数码管'
                readable = {}
            else:
                sensor = '未知'

            client_types[addr_str] = sensor

            socketio.emit('dashboard_data', {
                'addr': addr_str,
                'type': sensor,
                'data': readable
            })
            socketio.emit('client_list', [{'addr': a, 'type': client_types.get(a, '未知')} for a in clients.keys()])

    except Exception as e:
        print(f"[TCP] {addr_str} 异常: {e}")
    finally:
        conn.close()
        clients.pop(addr_str, None)
        client_types.pop(addr_str, None)
        socketio.emit('log', {'message': f'客户端 {addr_str} 已断开连接'})
        socketio.emit('client_list', [{'addr': a, 'type': client_types.get(a, '未知')} for a in clients.keys()])
        socketio.emit('disconnected', {'addr': addr_str})

def tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 8081))
    server.listen(5)
    print("[TCP] 服务器启动，等待连接...")

    while True:
        conn, addr = server.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == '__main__':
    threading.Thread(target=tcp_server, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000)