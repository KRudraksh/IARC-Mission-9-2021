import pyrebase

config = {
  "apiKey": "AIzaSyAiBNZfad7r3YewpcqYjc4ko1u7fd2s9yE",
  "authDomain": "iarc-b9c0a.firebaseapp.com",
  "databaseURL": "https://iarc-b9c0a-default-rtdb.firebaseio.com/",
  "storageBucket": "iarc-b9c0a.appspot.com"
}

firebase = pyrebase.initialize_app(config)
storage = firebase.storage()
database = firebase.database()
while True:
    accidents = database.child("/").get()
    data = accidents.val() 
    print(data)