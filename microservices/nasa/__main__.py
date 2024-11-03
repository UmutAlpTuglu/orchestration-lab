import argparse
import logging
import os
from dotenv import load_dotenv

from .get_data import get_data

# load password config file for local development
if os.path.exists("./.env"):
    load_dotenv("./.env")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Receive data from Nasa."
    )
    parser.add_argument(
        "--api_key",
        type=str,
        help="Nasa Api Key",
        default=os.getenv("NASA_API") or os.environ.get("NASA_API"),
    )
    parser.add_argument(
        "--nasa_url",
        type=str,
        help="Nasa Url to scrape image from",
        default=os.getenv("NASA_URL") or os.environ.get("NASA_URL"),
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

    get_data(args.api_key, args.nasa_url)