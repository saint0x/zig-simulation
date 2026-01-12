// Velocity limiter to prevent sudden jerky movements

export interface VelocityLimiterConfig {
  maxVelocity: number      // Maximum change per second
  maxAcceleration: number  // Maximum velocity change per second
}

export class VelocityLimiter {
  private currentValue: number = 0
  private currentVelocity: number = 0
  private lastUpdateTime: number = 0
  private config: VelocityLimiterConfig
  private initialized: boolean = false

  constructor(config: Partial<VelocityLimiterConfig> = {}) {
    this.config = {
      maxVelocity: config.maxVelocity ?? 100,      // degrees/second
      maxAcceleration: config.maxAcceleration ?? 200, // degrees/second^2
    }
  }

  update(targetValue: number, timestamp: number = Date.now()): number {
    if (!this.initialized) {
      this.currentValue = targetValue
      this.lastUpdateTime = timestamp
      this.initialized = true
      return targetValue
    }

    const dt = Math.max(0.001, (timestamp - this.lastUpdateTime) / 1000) // seconds
    this.lastUpdateTime = timestamp

    // Calculate desired velocity
    const desiredVelocity = (targetValue - this.currentValue) / dt

    // Limit velocity
    let limitedVelocity = Math.max(
      -this.config.maxVelocity,
      Math.min(this.config.maxVelocity, desiredVelocity)
    )

    // Limit acceleration
    const velocityChange = limitedVelocity - this.currentVelocity
    const maxVelocityChange = this.config.maxAcceleration * dt
    if (Math.abs(velocityChange) > maxVelocityChange) {
      limitedVelocity = this.currentVelocity + Math.sign(velocityChange) * maxVelocityChange
    }

    // Update state
    this.currentVelocity = limitedVelocity
    this.currentValue = this.currentValue + limitedVelocity * dt

    return this.currentValue
  }

  getValue(): number {
    return this.currentValue
  }

  getVelocity(): number {
    return this.currentVelocity
  }

  reset(value?: number): void {
    this.currentValue = value ?? 0
    this.currentVelocity = 0
    this.lastUpdateTime = 0
    this.initialized = value !== undefined
  }

  setConfig(config: Partial<VelocityLimiterConfig>): void {
    if (config.maxVelocity !== undefined) {
      this.config.maxVelocity = config.maxVelocity
    }
    if (config.maxAcceleration !== undefined) {
      this.config.maxAcceleration = config.maxAcceleration
    }
  }
}

// Multi-value velocity limiter
export class MultiVelocityLimiter {
  private limiters: Map<string, VelocityLimiter> = new Map()
  private config: VelocityLimiterConfig

  constructor(config: Partial<VelocityLimiterConfig> = {}) {
    this.config = {
      maxVelocity: config.maxVelocity ?? 100,
      maxAcceleration: config.maxAcceleration ?? 200,
    }
  }

  update(key: string, value: number, timestamp?: number): number {
    if (!this.limiters.has(key)) {
      this.limiters.set(key, new VelocityLimiter(this.config))
    }
    return this.limiters.get(key)!.update(value, timestamp)
  }

  updateAll(values: Record<string, number>, timestamp?: number): Record<string, number> {
    const result: Record<string, number> = {}
    for (const [key, value] of Object.entries(values)) {
      result[key] = this.update(key, value, timestamp)
    }
    return result
  }

  reset(): void {
    this.limiters.clear()
  }

  setConfig(config: Partial<VelocityLimiterConfig>): void {
    this.config = { ...this.config, ...config }
    for (const limiter of this.limiters.values()) {
      limiter.setConfig(config)
    }
  }
}
