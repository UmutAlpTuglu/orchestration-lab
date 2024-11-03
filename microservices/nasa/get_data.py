import logging
import os
import requests


def get_data(api_key : str, url:str) -> None:
    """ Get different NASA data

    Args:
        api_key (str): Nasa API key
        url (str): Nasa Url to scrape image from
    """

    apod_url = url
    params = {"api_key": api_key}

    response = requests.get(apod_url, params=params)
    
    if response.status_code != 200:
        logging.error(f"Failed to fetch data: {response.status_code}")
        if response.status_code == 403:
            logging.error("API key may be invalid or missing")
        return
    
    # If successful response
    data = response.json()
    extracted_data = {
        "date": data["date"],
        "explanation": data["explanation"],
        "title": data["title"],
        "url": data["url"],
    }
    logging.info(extracted_data)

    # Create data directory if it doesn't exist
    os.makedirs('data', exist_ok=True)

    image_url = extracted_data['url']
    response = requests.get(image_url, stream=True)
    if response.status_code == 200:
        
        filename = os.path.join('data', image_url.split("/")[-1])

        with open(filename, 'wb') as file:
            for chunk in response.iter_content(chunk_size=128):
                file.write(chunk)
        print(f"Image successfully downloaded and saved as {filename}")
    else:
        print(f"Failed to download image: {response.status_code}")