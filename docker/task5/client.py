import asyncio
import websockets
import json
import datetime
import os
import time
import socket


LOG_FILE = "/app/logs/websocket.log"

def log_message(message):
    """メッセージをログファイルに書き込む"""
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(f"[{timestamp}] [CLIENT] {message}\n")
    print(f"[{timestamp}] [CLIENT] {message}")

async def send_messages():
    """サーバーにメッセージを送信"""
    uri = "ws://server:8765"

    log_message("サーバーへの接続を試行中...")

    max_retries = 10
    retry_count = 0

    while retry_count < max_retries:
        try:
            async with websockets.connect(uri) as websocket:
                log_message(f"サーバーに接続しました: {uri}")

                messages = [
                    "こんにちは、サーバー！",
                    "WebSocket通信のテストです",
                    "Docker Composeで動作中",
                    "コンテナ間通信が成功しました",
                    "最後のメッセージです"
                ]

                for i, msg in enumerate(messages, 1):
                    message_data = {
                        "message": msg,
                        "message_id": i,
                        "timestamp": datetime.datetime.now().isoformat()
                    }

                    await websocket.send(json.dumps(message_data, ensure_ascii=False))
                    log_message(f"送信 [{i}]: {msg}")

                    response = await websocket.recv()
                    log_message(f"受信 [{i}]: {response}")

                    await asyncio.sleep(1)

                log_message("全メッセージの送信が完了しました")
                break

        except ConnectionRefusedError:
            retry_count += 1
            log_message(f"接続に失敗しました。再試行中... ({retry_count}/{max_retries})")
            await asyncio.sleep(2)
        except socket.gaierror as e:
            retry_count += 1
            log_message(f"アドレス解決に失敗しました。再試行中... ({retry_count}/{max_retries}) エラー: {e}")
            await asyncio.sleep(2)
        except Exception as e:
            log_message(f"エラーが発生しました: {e}")
            break

    if retry_count >= max_retries:
        log_message("サーバーへの接続に失敗しました。最大再試行回数に達しました。")

async def main():
    """クライアントのメイン処理"""
    log_message("WebSocketクライアントを開始します")
    await send_messages()
    log_message("WebSocketクライアントを終了します")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log_message("クライアントが停止されました")
