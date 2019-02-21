package org.ros.android.android_wear_pub

import android.content.Intent
import android.os.Bundle
import android.support.wearable.activity.WearableActivity

class MainActivity: WearableActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        //check why doesn't work
        val intent = Intent(this, Proximity::class.java)
        startActivity(intent)
        finish()
    }
}