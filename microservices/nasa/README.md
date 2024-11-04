# Nasa API App

Go to this website to create a [Nasa API KEY](https://api.nasa.gov/).

For local development you have to create a `.env` file at top level like this and paste your API Key:
```text
NASA_API=
NASA_URL=https://api.nasa.gov/planetary/apod
```
The NASA_URL is optional, per default it uses the planetary/apod. For other APIs you need to configure the query parameters differently.

This app just scrapes the latest image from the API (e.g. a Saturn moon) and saves it in data folder at top level.
