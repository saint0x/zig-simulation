// Simple 1D Kalman Filter for smoothing hand tracking data

export interface KalmanState {
  x: number      // Estimated value
  p: number      // Estimation error covariance
  q: number      // Process noise covariance
  r: number      // Measurement noise covariance
}

export class KalmanFilter {
  private state: KalmanState

  constructor(
    initialValue: number = 0,
    processNoise: number = 0.01,
    measurementNoise: number = 0.1
  ) {
    this.state = {
      x: initialValue,
      p: 1,
      q: processNoise,
      r: measurementNoise,
    }
  }

  update(measurement: number): number {
    // Prediction
    const p_pred = this.state.p + this.state.q

    // Update
    const k = p_pred / (p_pred + this.state.r) // Kalman gain
    this.state.x = this.state.x + k * (measurement - this.state.x)
    this.state.p = (1 - k) * p_pred

    return this.state.x
  }

  getValue(): number {
    return this.state.x
  }

  reset(value: number = 0): void {
    this.state.x = value
    this.state.p = 1
  }

  setNoiseParams(processNoise: number, measurementNoise: number): void {
    this.state.q = processNoise
    this.state.r = measurementNoise
  }
}

// Multi-dimensional Kalman filter for tracking multiple values
export class MultiKalmanFilter {
  private filters: Map<string, KalmanFilter> = new Map()
  private processNoise: number
  private measurementNoise: number

  constructor(processNoise: number = 0.01, measurementNoise: number = 0.1) {
    this.processNoise = processNoise
    this.measurementNoise = measurementNoise
  }

  update(key: string, measurement: number): number {
    if (!this.filters.has(key)) {
      this.filters.set(key, new KalmanFilter(measurement, this.processNoise, this.measurementNoise))
      return measurement
    }
    return this.filters.get(key)!.update(measurement)
  }

  updateAll(measurements: Record<string, number>): Record<string, number> {
    const result: Record<string, number> = {}
    for (const [key, value] of Object.entries(measurements)) {
      result[key] = this.update(key, value)
    }
    return result
  }

  getValue(key: string): number | undefined {
    return this.filters.get(key)?.getValue()
  }

  reset(): void {
    this.filters.clear()
  }

  setNoiseParams(processNoise: number, measurementNoise: number): void {
    this.processNoise = processNoise
    this.measurementNoise = measurementNoise
    for (const filter of this.filters.values()) {
      filter.setNoiseParams(processNoise, measurementNoise)
    }
  }
}
