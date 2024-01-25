# select a base image
FROM waggle/plugin-base:1.1.1-base

# put all of our apps code in /app
WORKDIR /app

# install all python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# copy rest of our code
COPY . .

# define how to run our code
ENTRYPOINT ["python3", "main.py"]
