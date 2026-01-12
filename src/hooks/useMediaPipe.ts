// MediaPipe Gesture Recognizer initialization hook

import { useState, useEffect, useRef, useCallback } from 'react'
import {
  GestureRecognizer,
  FilesetResolver,
} from '@mediapipe/tasks-vision'
import type { MediaPipeConfig, UseMediaPipeReturn } from '@/types/mediapipe'
import { DEFAULT_MEDIAPIPE_CONFIG, MODEL_PATHS } from '@/lib/mediapipe/config'

export function useMediaPipe(
  config: Partial<MediaPipeConfig> = {}
): UseMediaPipeReturn & { gestureRecognizer: GestureRecognizer | null } {
  const [isInitialized, setIsInitialized] = useState(false)
  const [isLoading, setIsLoading] = useState(false)
  const [error, setError] = useState<Error | null>(null)

  const gestureRecognizerRef = useRef<GestureRecognizer | null>(null)
  const initializingRef = useRef(false)

  const mergedConfig = { ...DEFAULT_MEDIAPIPE_CONFIG, ...config }

  const initializeMediaPipe = useCallback(async () => {
    if (initializingRef.current || gestureRecognizerRef.current) {
      return
    }

    initializingRef.current = true
    setIsLoading(true)
    setError(null)

    try {
      // Initialize the vision task
      const vision = await FilesetResolver.forVisionTasks(MODEL_PATHS.wasmPath)

      // Create the gesture recognizer
      const recognizer = await GestureRecognizer.createFromOptions(vision, {
        baseOptions: {
          modelAssetPath: MODEL_PATHS.gestureRecognizer,
          delegate: 'GPU',
        },
        runningMode: 'VIDEO',
        numHands: mergedConfig.numHands,
        minHandDetectionConfidence: mergedConfig.minDetectionConfidence,
        minTrackingConfidence: mergedConfig.minTrackingConfidence,
      })

      gestureRecognizerRef.current = recognizer
      setIsInitialized(true)
      console.log('MediaPipe GestureRecognizer initialized successfully')
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to initialize MediaPipe')
      setError(error)
      console.error('MediaPipe initialization error:', error)
    } finally {
      setIsLoading(false)
      initializingRef.current = false
    }
  }, [mergedConfig.numHands, mergedConfig.minDetectionConfidence, mergedConfig.minTrackingConfidence])

  // Initialize on mount
  useEffect(() => {
    initializeMediaPipe()

    return () => {
      if (gestureRecognizerRef.current) {
        gestureRecognizerRef.current.close()
        gestureRecognizerRef.current = null
        setIsInitialized(false)
      }
    }
  }, [initializeMediaPipe])

  return {
    isInitialized,
    isLoading,
    error,
    gestureRecognizer: gestureRecognizerRef.current,
  }
}
