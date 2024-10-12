package com.example.spotfinder

import com.google.gson.GsonBuilder
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import retrofit2.http.GET
import retrofit2.Response
import retrofit2.http.Body
import retrofit2.http.POST

// define API enpoints
interface ApiService {
    @GET("/get_vehicle_status")
    suspend fun getVehicleStatus(): Response<VehicleStatus>

    @POST("/login")
    suspend fun login(@Body userData: UserData): Response<PostResponse>

    @POST("/sign_in")
    suspend fun signIn(@Body userData: UserData): Response<PostResponse>
}

// Create a Retrofit instance
object RetrofitInstance {
    private const val BASE_URL = "http://10.199.1.121:8080"

    val gson = GsonBuilder().setLenient().create()

    val api: ApiService by lazy {
        Retrofit.Builder()
            .baseUrl(BASE_URL)
            .addConverterFactory(GsonConverterFactory.create(gson))
            .build()
            .create(ApiService::class.java)
    }
}