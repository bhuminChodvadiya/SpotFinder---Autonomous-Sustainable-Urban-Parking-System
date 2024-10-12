package com.example.spotfinder

data class VehicleStatus(
    val xCoordinate: Float,
    val yCoordinate: Float,
    val parkingPositionReached: Boolean
)

data class UserData(
    val email: String,
    val password: String
)

data class PostResponse(
    val success: Boolean,
    val message: String
)