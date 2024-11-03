import argparse
import logging
import os
from dotenv import load_dotenv

from microservices.nasa.get_data import get_data

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
        default=os.getenv("NASA_URL") or os.environ.get("NASA_URL") or "https://api.nasa.gov/planetary/apod",
    )
    parser.add_argument(
        "--debug", help="Set if you want to see debug messages", action="store_true"
    )

    args = parser.parse_args()

    print(f"NASA_API env var: {os.getenv('NASA_API')}")
    print(f"API_KEY env var: {os.getenv('API_KEY')}")
    print(f"Final api_key value: {args.api_key}")

    fmt = "%(asctime)s %(levelname)s %(message)s"
    logging.captureWarnings(True)
    
    logging.basicConfig(level=logging.DEBUG, format=fmt)

    get_data(args.api_key, args.nasa_url)