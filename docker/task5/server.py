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
        f.write(f"[{timestamp}] [SERVER] {message}\n")
    print(f"[{timestamp}] [SERVER] {message}")

async def handle_client(websocket):
    """クライアントからの接続を処理"""
    client_address = websocket.remote_address
    log_message(f"新しいクライアントが接続しました: {client_address}")

    try:
        async for message in websocket:
            try:
                # JSONメッセージをパース
                data = json.loads(message)
                msg_content = data.get("message", "")
                msg_id = data.get("message_id", 0)
                timestamp = data.get("timestamp", "")

                log_message(f"受信 [{msg_id}] from {client_address}: {msg_content}")

                response = {
                    "response": f"サーバーが受信しました: {msg_content}",
                    "original_message_id": msg_id,
                    "server_timestamp": datetime.datetime.now().isoformat(),
                    "status": "success"
                }

                await websocket.send(json.dumps(response, ensure_ascii=False))
                log_message(f"送信 [{msg_id}] to {client_address}: 応答メッセージ送信完了")

            except json.JSONDecodeError:
                log_message(f"テキストメッセージ受信 from {client_address}: {message}")
                response = f"サーバーが受信: {message}"
                await websocket.send(response)

    except websockets.exceptions.ConnectionClosed:
        log_message(f"クライアント {client_address} が切断しました")
    except Exception as e:
        log_message(f"エラーが発生しました (client: {client_address}): {e}")

async def main():
    """WebSocketサーバーのメイン処理"""
    host = "0.0.0.0"
    port = 8765

    log_message("WebSocketサーバーを起動します")
    log_message(f"サーバーアドレス: {host}:{port}")

    async with websockets.serve(handle_client, host, port):
        log_message("WebSocketサーバーが起動しました。クライアントからの接続を待機中...")

        await asyncio.Future()  # 無限に実行

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log_message("サーバーが停止されました")
    except Exception as e:
        log_message(f"サーバーエラー: {e}")
