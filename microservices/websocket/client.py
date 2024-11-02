"""Defines handlers for websocket connection"""

import logging
from typing import Any

import websocket


def on_error(ws: websocket.WebSocketApp, error: Any) -> None:
    """Called when websocket error has occurred"""
    logging.error("Websocket error")
    logging.error(error)


def on_close(
    ws: websocket.WebSocketApp, close_status_code: int, close_msg: str
) -> None:
    """Called when websocket is closed"""
    logging.error("Websocket closed")
    logging.error(f"Status code {close_status_code}")
    logging.error(f"Close msg {close_msg}")

def run_feedhandler(
    hostname: str,
    port: int,
) -> None:
    """Connects to ... via Websockets and publishes data to Kafka in infinite loop

    Args:
        hostname (str): 
        kafka_server (str): Adress of Kafka server, e. g. "kafka:9092"
    """
    # Create message handler
    #message_processor = MessageProcessor(kafka_server)

    # Connect to websocket Server
    # ws_address = f"ws://{hostname}:{port}/WebSocket"
    # logging.info(f"Connecting to WebSocket {ws_address} ...")
    # web_socket_app = websocket.WebSocketApp(
    #     ws_address,
    #     header=["User-Agent: Python"],
    #     # If the on_message callback is invoked, drop the ws object and process msg
    #     #on_message=lambda ws, msg: message_processor.process_message(msg),
    #     on_error=on_error,
    #     on_close=on_close,
    # )
    # web_socket_app.run_forever(
    #     ping_interval=None,
    #     ping_timeout=10,
    # )