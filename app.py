# server.py
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import threading
import socket

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

clients = {}  # {addr_str: conn}
client_types = {}  # {addr_str: sensor_type}

@app.route('/')
def index():
    return render_template('index.html')

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

            #msg = list(data)
            #print(f"[TCP] 收到 {addr_str}: {msg}")
            
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
            
            temp = bytes(payload[:2]).decode(errors='ignore')
            humi = bytes(payload[2:4]).decode(errors='ignore')
            #print(temp, humi);
            
            if sensor_type == 0xa0:
                sensor = '温湿度'
                temp = bytes(payload[:2]).decode(errors='ignore')
                humi = bytes(payload[2:4]).decode(errors='ignore')
                readable = {'temp': temp, 'humi': humi}
            elif sensor_type == 0xa1:
                sensor = '蜂鸣器'
                readable = {'status': '已开启' if payload[0] == 1 else '关闭'}
            elif sensor_type == 0xa2:
                sensor = '风扇'
                readable = {'status': '已开启' if payload[0] == 1 else '关闭'}
            elif sensor_type == 0xa3:
                sensor = '光照'
                readable = {}
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