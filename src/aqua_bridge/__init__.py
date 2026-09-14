"""aqua-bridge: MPC fan controller for aquaero 6 XT + Quadro on a Raspberry Pi.

Package layout (see PROJECT.md section 3):

- ``aqua_bridge.model``     -- the contract: observation, config, command, state
- ``aqua_bridge.config``    -- YAML loading and validation
- ``aqua_bridge.control``   -- sensor gate, solvers, intents, glue loop
- ``aqua_bridge.hw``        -- hidraw and 1-Wire adapters (never imports control)
- ``aqua_bridge.sim``       -- RC thermal plant for closed-loop tests
- ``aqua_bridge.publishers`` -- HTTP and MQTT views over (obs, cmd)
"""

__version__ = "0.1.0"
