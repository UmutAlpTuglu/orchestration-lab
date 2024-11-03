#!/usr/bin/env python
# python3 -m microservices.websocket
import argparse
import logging

from .client import run_feedhandler

if __name__ == "__main__":
    # Define command line arguments
    parser = argparse.ArgumentParser(description="Feed data from WebServer to WebClient")
    parser.add_argument(
        "--hostname",
        type=str,
        help="Test WebServer",
        default="",
    )
    parser.add_argument(
        "--debug", help="Set if you want to see debug messages", action="store_true"
    )

    args = parser.parse_args()


    # Setup logging format and enable logging of warnings.
    fmt = "%(asctime)s %(levelname)s %(message)s"
    logging.captureWarnings(True)

    # Log debug logging only if --debug is set.
    if args.debug:
        logging.basicConfig(level=logging.DEBUG, format=fmt)
    else:
        logging.basicConfig(level=logging.INFO, format=fmt)

    # Call to main program
    logging.info(f"passed commandline arguments :{args}")
    run_feedhandler(
        args.kafka_server
    )