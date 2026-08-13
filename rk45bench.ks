runoncepath("0:/den4ick240kos/boosterlib.ks").

// Run the integrator benchmark from the current flight state.
// Tune rk45Rtol until "accuracy matches rk4" prints true, then compare the
// force evals / game-time spent to see if rk45 would be faster.
// Usage: compareIntegrators(simVelocity, rk45Rtol, rk45Atol) for custom tuning.
compareIntegrators(ship:velocity:orbit, 0.1, 0.001).
