// Real-time hand tracking hook using MediaPipe

import { useState, useRef, useCallback, useEffect } from 'react'
import type { GestureRecognizer, GestureRecognizerResult } from '@mediapipe/tasks-vision'
import type {
  ProcessedHandData,
  GestureResult,
  UseHandTrackingReturn,
  NormalizedLandmark,
} from '@/types/mediapipe'
import { processLandmarks } from '@/lib/mediapipe/mapping'

interface UseHandTrackingOptions {
  enabled?: boolean
  mirrorMode?: boolean
  onLandmarksUpdate?: (data: ProcessedHandData) => void
  onGestureDetected?: (gesture: GestureResult) => void
}

export function useHandTracking(
  videoRef: React.RefObject<HTMLVideoElement>,
  gestureRecognizer: GestureRecognizer | null,
  options: UseHandTrackingOptions = {}
): UseHandTrackingReturn {
  const {
    enabled = true,
    mirrorMode = true,
    onLandmarksUpdate,
    onGestureDetected,
  } = options

  const [isTracking, setIsTracking] = useState(false)
  const [handData, setHandData] = useState<ProcessedHandData | null>(null)
  const [gesture, setGesture] = useState<GestureResult | null>(null)
  const [fps, setFps] = useState(0)

  const animationFrameRef = useRef<number | null>(null)
  const lastVideoTimeRef = useRef<number>(-1)
  const frameCountRef = useRef(0)
  const lastFpsUpdateRef = useRef(Date.now())
  const trackingRef = useRef(false)

  const processFrame = useCallback(() => {
    if (!trackingRef.current || !videoRef.current || !gestureRecognizer) {
      animationFrameRef.current = requestAnimationFrame(processFrame)
      return
    }

    const video = videoRef.current

    // Only process if video time has changed (new frame)
    if (video.currentTime === lastVideoTimeRef.current) {
      animationFrameRef.current = requestAnimationFrame(processFrame)
      return
    }

    lastVideoTimeRef.current = video.currentTime

    try {
      const nowMs = performance.now()
      const result: GestureRecognizerResult = gestureRecognizer.recognizeForVideo(video, nowMs)

      // Update FPS counter
      frameCountRef.current++
      const now = Date.now()
      if (now - lastFpsUpdateRef.current >= 1000) {
        setFps(frameCountRef.current)
        frameCountRef.current = 0
        lastFpsUpdateRef.current = now
      }

      // Process results
      if (result.landmarks && result.landmarks.length > 0) {
        const landmarks = result.landmarks[0] as NormalizedLandmark[]
        const handedness = result.handednesses?.[0]?.[0]?.categoryName as 'Left' | 'Right' ?? 'Right'
        const confidence = result.handednesses?.[0]?.[0]?.score ?? 0.5

        // Process landmarks into structured data
        const processed = processLandmarks(landmarks, handedness, confidence, mirrorMode)
        setHandData(processed)
        onLandmarksUpdate?.(processed)

        // Process gestures
        if (result.gestures && result.gestures.length > 0 && result.gestures[0].length > 0) {
          const topGesture = result.gestures[0][0]
          const gestureResult: GestureResult = {
            categoryName: topGesture.categoryName,
            score: topGesture.score,
            index: topGesture.index,
          }
          setGesture(gestureResult)
          onGestureDetected?.(gestureResult)
        } else {
          setGesture(null)
        }
      } else {
        setHandData(null)
        setGesture(null)
      }
    } catch (err) {
      console.error('Hand tracking error:', err)
    }

    animationFrameRef.current = requestAnimationFrame(processFrame)
  }, [videoRef, gestureRecognizer, mirrorMode, onLandmarksUpdate, onGestureDetected])

  const startTracking = useCallback(() => {
    if (!gestureRecognizer || trackingRef.current) return

    trackingRef.current = true
    setIsTracking(true)
    lastVideoTimeRef.current = -1
    frameCountRef.current = 0
    lastFpsUpdateRef.current = Date.now()

    animationFrameRef.current = requestAnimationFrame(processFrame)
    console.log('Hand tracking started')
  }, [gestureRecognizer, processFrame])

  const stopTracking = useCallback(() => {
    trackingRef.current = false
    setIsTracking(false)

    if (animationFrameRef.current) {
      cancelAnimationFrame(animationFrameRef.current)
      animationFrameRef.current = null
    }

    setHandData(null)
    setGesture(null)
    setFps(0)
    console.log('Hand tracking stopped')
  }, [])

  // Handle enabled prop changes
  useEffect(() => {
    if (enabled && gestureRecognizer && videoRef.current) {
      // Don't auto-start - let user explicitly start
    } else if (!enabled) {
      stopTracking()
    }
  }, [enabled, gestureRecognizer, videoRef, stopTracking])

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current)
      }
    }
  }, [])

  return {
    isTracking,
    handData,
    gesture,
    fps,
    startTracking,
    stopTracking,
  }
}
