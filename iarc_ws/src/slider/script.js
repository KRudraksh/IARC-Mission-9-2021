var firebaseConfig = {
    apiKey: "AIzaSyAiBNZfad7r3YewpcqYjc4ko1u7fd2s9yE",
    authDomain: "iarc-b9c0a.firebaseapp.com",
    projectId: "iarc-b9c0a",
    storageBucket: "iarc-b9c0a.appspot.com",
    messagingSenderId: "513581085766",
    appId: "1:513581085766:web:f72ea8e7e092a55d3c4647",
    measurementId: "G-WN9TB2GJC3"
  };
  // Initialize Firebase
  firebase.initializeApp(firebaseConfig);
  // firebase.analytics();
  var database = firebase.database()

new Vue(

{
  el: '#app',
  data() {
    return {
      password: '',
      copied: false,
      settings: {
        max_wind_speed: 150,
        maxDegs: 360,
        maxStates: 5,
        maxFlight_speed: 150,
        maxEndurance: 15,
        wind_speed: 0,
        degs: 0,
        states: 0  ,
        flight_speed: 0  ,
        endurance: 0  ,
        ambiguous: true,
      }
    };
  },
  computed: {
    wind_speedThumbPosition: function() {
      var current_wind_speed =  (( (this.settings.wind_speed - 0) / (this.settings.max_wind_speed - 0)) * 100);
      database.ref("wind_speed").set(current_wind_speed)
      return current_wind_speed
    },
    degsThumbPosition: function() {
      var current_degs =  (( (this.settings.degs - 0) / (this.settings.maxDegs - 0)) * 100);
      database.ref("degs").set(current_degs)
      return current_degs
    },
    statesThumbPosition: function() {
      var current_sea_state = (( (this.settings.states - 0) / (this.settings.maxStates - 0)) * 100);
      database.ref("states").set(current_sea_state)      
      return current_sea_state
    },
    flight_speedThumbPosition: function() {
      var current_flight_speed =  (( (this.settings.flight_speed - 0) / (this.settings.maxFlight_speed - 0)) * 100);
      database.ref("flight_speed").set(current_flight_speed)
      return current_flight_speed

    },
    enduranceThumbPosition: function() {
      var current_endurance =  (( (this.settings.endurance - 0) / (this.settings.maxEndurance - 0)) * 100);
      database.ref("endurance").set(current_endurance)
      return current_endurance
    },
  
  },
});