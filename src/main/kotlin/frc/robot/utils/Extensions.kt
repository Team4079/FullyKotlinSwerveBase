@file:Suppress("unused")

package frc.robot.utils

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString

fun dashNumbers(vararg pairs: Pair<String, Number>) =
  pairs.forEach { (key, value) -> putNumber(key, value.toDouble()) }

fun dashStrings(vararg pairs: Pair<String, String>) =
  pairs.forEach { (key, value) -> putString(key, value) }
