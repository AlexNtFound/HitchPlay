package com.chatgptlite.wanted

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.asSharedFlow

/**
 * Used to communicate between screens.
 */
class MainViewModel : ViewModel() {
    private val _drawerShouldBeOpened = MutableStateFlow(false)
    val drawerShouldBeOpened = _drawerShouldBeOpened.asStateFlow()

    fun openDrawer() {
        _drawerShouldBeOpened.value = true
    }

    fun resetOpenDrawerAction() {
        _drawerShouldBeOpened.value = false
    }

    // Broadcast rover state changes
    private val _roverStateFlow = MutableSharedFlow<String>(replay = 1)
    val roverStateFlow = _roverStateFlow.asSharedFlow() // Expose as read-only

    // Function to update rover state
    suspend fun setRoverState(newState: String) {
        _roverStateFlow.emit(newState) // Emit new state to observers
    }
}