#!/usr/bin/env python3
import asyncio
import websockets
import json
import datetime
import os

# ログファイルのパス
LOG_FILE = "/app/logs/websocket.log"

def log_message(message):
    """メッセージをログファイルに書き込む"""
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(f"[{timestamp}] {message}\n")
    print(f"[{timestamp}] {message}")

async def handle_client(websocket, path):
    """クライアントからの接続を処理"""
    client_address = websocket.remote_address
    log_message(f"クライアントが接続しました: {client_address}")

    try:
        async for message in websocket:
            # 受信したメッセージをログに記録
            log_message(f"受信: {message}")

            # JSONとしてパース
            try:
                data = json.loads(message)
                client_message = data.get("message", "")

                # 応答メッセージを作成
                response = {
                    "timestamp": datetime.datetime.now().isoformat(),
                    "server_message": f"サーバーが受信しました: {client_message}",
                    "status": "success"
                }

                # クライアントに応答を送信
                await websocket.send(json.dumps(response, ensure_ascii=False))
                log_message(f"送信: {json.dumps(response, ensure_ascii=False)}")

            except json.JSONDecodeError:
                # JSONでない場合のエラー応答
                error_response = {
                    "timestamp": datetime.datetime.now().isoformat(),
                    "error": "Invalid JSON format",
                    "status": "error"
                }
                await websocket.send(json.dumps(error_response))
                log_message(f"エラー送信: Invalid JSON format")

    except websockets.exceptions.ConnectionClosed:
        log_message(f"クライアントが切断しました: {client_address}")
    except Exception as e:
        log_message(f"エラーが発生しました: {e}")

async def main():
    """WebSocketサーバーを起動"""
    host = "0.0.0.0"
    port = 8765

    log_message(f"WebSocketサーバーを起動します: ws://{host}:{port}")

    # WebSocketサーバーを開始
    server = await websockets.serve(handle_client, host, port)
    log_message("WebSocketサーバーが起動しました")

    # サーバーを永続的に実行
    await server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log_message("サーバーが停止されました")
