package com.example.spotfinder

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.activity.ComponentActivity
import androidx.activity.enableEdgeToEdge
import androidx.core.app.ActivityCompat
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat
import android.Manifest
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Paint
import android.os.Handler
import android.os.Looper

import android.view.View
import android.widget.ImageView
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import java.security.MessageDigest

class MainActivity : ComponentActivity() {
    //initializing global variables
    // initializing a handler for periodic tasks running in the background
    private val handler = Handler(Looper.getMainLooper())
    // interval of 1 second for sending periodically requests
    private val interval = 1000L
    // creating an instance of the API interface
    private val api = RetrofitInstance.api

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        // set the initial view to login page
        setContentView(R.layout.login)

        createNotificationChannel()
        setupLoginButtons()
    }

    // function to set up event listeners for Buttons on the Login-page
    private  fun setupLoginButtons(){
        findViewById<Button>(R.id.btLogin).setOnClickListener {
            // getting the email and password text field
            val txEmail = findViewById<TextView>(R.id.txEmail)
            val txPassword = findViewById<TextView>(R.id.txPassword)

            // extracting text values from the text fields and encrypt the password
            val email = txEmail.text.toString()
            val password = hashPassword(txPassword.text.toString())

            CoroutineScope(Dispatchers.IO).launch {
                try{
                    // create userData element for sending the request
                    val userData = UserData(email=email, password=password)

                    // sending login request
                    val loginResponse = api.login(userData)

                    if (loginResponse.isSuccessful) {
                        runOnUiThread {
                            // set current view to map view
                            setContentView(R.layout.map)
                            startPeriodicRequests()
                            setupMapButtons()
                        }
                    }else{
                        runOnUiThread {
                            // show error message and reset text fields
                            val txError = findViewById<TextView>(R.id.txError)
                            txError.text = "invalid email or password"
                            txEmail.text = ""
                            txPassword.text = ""
                        }
                    }
                } catch (e: Exception) {
                    Log.e("sendRequest", "Network request failed", e)
                }
            }
        }

        findViewById<Button>(R.id.btSignIn).setOnClickListener {
            // set current view to sign-in view
            setContentView(R.layout.sign_in)
            setupSignInButtons()
        }
    }

    // function to set up event listeners for Buttons on the Sign-In-page
    private fun setupSignInButtons() {
        findViewById<Button>(R.id.btCreateAccount).setOnClickListener {
            // extracting text values from the text fields and encrypt the password
            val email = findViewById<TextView>(R.id.txRegisterEmail).text.toString()
            val password = hashPassword(findViewById<TextView>(R.id.txRegisterPassword).text.toString())
            CoroutineScope(Dispatchers.IO).launch {
                try{
                    // create userData element for sending the request
                    val userData = UserData(email=email, password=password)
                    // call the Post-request login
                    val signInResponse = api.signIn(userData)

                    if (signInResponse.isSuccessful) {
                        runOnUiThread {
                            // set current view to login view
                            setContentView(R.layout.login)
                            setupLoginButtons()
                        }
                    } else {
                        runOnUiThread {
                            // show error message
                            val txError = findViewById<TextView>(R.id.txError)
                            txError.text = "email and password cannot be empty"
                        }
                    }
                } catch (e: Exception) {
                    Log.e("sendRequest", "Network request failed", e)
                }
            }
        }
    }

    // function to set up event listeners for Buttons on the Map-page
    private fun setupMapButtons() {
        findViewById<Button>(R.id.btLogout).setOnClickListener {
            // set current view to login view
            setContentView(R.layout.login)
            setupLoginButtons()
        }
    }

    // function to start periodic requests using a handler
    private fun startPeriodicRequests() {
        handler.post(object : Runnable {
            override fun run() {
                sendRequest()
                // scheduling the next function call
                handler.postDelayed(this, interval)
            }
        })
    }

    // function to send network request to get vehicle status
    private fun sendRequest() {
        CoroutineScope(Dispatchers.IO).launch {
            try {
                // sending request to get vehicle status and extract response
                val response = RetrofitInstance.api.getVehicleStatus()

                if (response.isSuccessful) {
                    // convert response from json-format to a vehicleStatus-object and call function
                    response.body()?.let { vehicleStatus -> handleVehicleStatus(vehicleStatus) }
                } else {
                    Log.e("sendRequest", "Request failed with code: ${response.code()}")
                }
            } catch (e: Exception) {
                Log.e("sendRequest", "Network request failed", e)
            }
        }
    }

    // function to handle vehicle status response and update UI accordingly
    private fun handleVehicleStatus(vehicleStatus: VehicleStatus) {
        // decoding the map image and create a bitmap
        val bitmap = BitmapFactory.decodeResource(resources, R.drawable.model_city_map).copy(Bitmap.Config.ARGB_8888, true)

        // create blue color paint
        val paint = Paint().apply {
            isAntiAlias = true
            color = -16776961
        }

        // calculate multiplication factor depending on the phone resolution
        val multFactorX = bitmap.height/812F * 100
        val multFactorY = bitmap.width/800F * 100

        // draw circle representing the current position of the vehicle
        val canvas = Canvas(bitmap)
        canvas.drawCircle(vehicleStatus.yCoordinate*multFactorY, vehicleStatus.xCoordinate*multFactorX, 50F, paint)

        // update image of the model city with the new position of the model car
        val imageView = findViewById<View>(R.id.imageView3) as ImageView
        imageView.adjustViewBounds = true
        imageView.setImageBitmap(bitmap)

        // if final parking position is reached, create pop-up notification
        if (vehicleStatus.parkingPositionReached){
            // stop periodic requests
            handler.removeMessages(0)
            notifyUser()
        }
    }

    // function to notify the user, when the vehicle has reached the parking spot
    private fun notifyUser(){

        val intent = Intent(this, MainActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
        }

        val pendingIntent: PendingIntent = PendingIntent.getActivity(this, 0, intent, PendingIntent.FLAG_IMMUTABLE)

        // create the notification object
        val builder = NotificationCompat.Builder(this, "notification_channel")
            .setSmallIcon(R.drawable.logo)
            .setContentTitle("Spotfinder")
            .setContentText("your vehicle has reached the parking spot")
            .setPriority(NotificationCompat.PRIORITY_DEFAULT)
            .setContentIntent(pendingIntent)
            .setAutoCancel(true)

        with(NotificationManagerCompat.from(this)) {
            // check if permission for post notifications is granted
            if (ActivityCompat.checkSelfPermission(
                    this@MainActivity,
                    Manifest.permission.POST_NOTIFICATIONS
                ) != PackageManager.PERMISSION_GRANTED
            ) {
                return@with
            }

            // display the notification
            notify(1, builder.build())
        }
    }

    // function to create a notification channel
    private fun createNotificationChannel() {

        val channel = NotificationChannel("notification_channel", "user notification", NotificationManager.IMPORTANCE_DEFAULT).apply {
            description = "notify the user that the vehicle has parked"
        }

        // register the channel in the system.
        val notificationManager: NotificationManager = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        notificationManager.createNotificationChannel(channel)
    }

    // function to encrypt a password using SHA-256
    private fun hashPassword(password: String): String {
        return MessageDigest.getInstance("SHA-256").digest(password.toByteArray()).joinToString("") { "%02x".format(it) }
    }
}