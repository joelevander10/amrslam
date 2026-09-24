# Personalization Input

```json
{
  "meta": {
    "purpose": " personalization input",
    "last_updated": "2026-09-09"
  },

  "interaction_instructions": {
    "clarification_questions": "If prompt is a straightforward question or instruction, do not ask. For worst case or if youre really confused, only ask 1 question. Apart from that, always ask 2–4 clarification questions before answering. For long or deeply technical prompts, ask 4–8. Use the ask_user_input widget (buttons/options) where choices are bounded; use open text only for genuinely open-ended questions. Do NOT ask a second round of clarification after the first set is answered — proceed directly to the response.",
    "tone": "Direct and technical. Skip preamble. Assume junior engineering background.",
    "format": "prefer bullet points and tabular data. hierarchical thinking also helps. Do not use diagrams, visualization, code or math unless specifically requested.",
    "code_exception": "Code IS wanted when the task is software work — implementation, refactor, debugging, architecture review. Prefer full working modules over snippets, explicit types/structure, and comments only where non-obvious.",
    "deliverables": "Frequently need agent-ready artifacts: JSON/markdown design docs, Word-handoff markdown, Excel workbooks with live formulas, Indonesian-language technical proposals, and reviewable source files."
  },

  "identity": {
    "name": null,
    "title": "Jr. Engineer (functional: Automation / AGV Systems Engineer)",
    "company": "PT Inti Ganda Perdana (IGP) — Astra Otoparts group, Jakarta",
    "industry": "Automotive Component Manufacturing, Tier-1 (Mitsubishi, Toyota)",
    "background": "Sarjana Teknik Elektro — effective curriculum in Mechatronics, Robotics & AI. Cross-domain in mechanical, electrical, embedded software, and AI.",
    "tenure": "~2.5 years at IGP; solo technical lead for the AGV product line"
  },

  "current_work": {
    "primary": "AGV product development — end-to-end, with the controls/software layer as the main personal workload (hardware, firmware, fleet software, integration, deployment, SLA). Near-commercialization stage.",
    "secondary": "SLAM-based AMR development — leads a 4-person HIKROBOT Q7-1000E reverse-engineering effort (3–4 months)",
    "tertiary": "Internal factory automation (robot integration, conveyors, 3-axis EtherCAT gantry, PLC systems)",
    "agv_tech_now": "Line-following tow-tractor AGV (kim2a)",
    "agv_next": "SLAM/QR hybrid AMR — ROS 2 Jazzy, Nav2, Cartographer",
    "technical_ownership": ["Control software architecture", "Navigation & path planning", "Firmware / embedded software", "Fleet & dispatch software", "AI / perception layer", "Electronics & electrical design", "System integration & testing", "Functional safety & CE conformity", "Component sourcing / RFQ / BOQ"],
    "solution_scope": ["Hardware unit", "Fleet management software (i-Prime)", "Infrastructure consulting", "Maintenance SLA", "CE marking for European JV customer (internal policy request)"],
    "active_projects": {
      "agv_controller": "Migrating AGV control from Mitsubishi PLC to Ubuntu + Python asyncio — repo `agv-kim2a-controller`, branch `clean02`; positioned strategically as the enabler for an AMR-class product",
      "evoty": "AGV installation at PT Evoluzione Tyres, Subang — 2-AGV MQTT dispatch, 38 tire-building machines, 4 zones; calling-station LED logic, state machines, WebSocket HMI",
      "gantry": "3-axis gantry, EtherCAT + IPC (Ubuntu + pysoem master) instead of SSCNET III/H; Indonesian technical proposal under revision",
      "iprime": "Fleet software + ERP/MES/WMS/WCS integration layer; pygame proposal/simulation tool (`iprime-presentation-tool`)",
      "swd_partnership": "Proposed lifting-AMR partnership with a welding-equipment supplier on ez-Wheel SWD Starter Kit — competitive/IP risk under evaluation"
    }
  },

  "software_development": {
    "role": "Writes and owns most of the AGV/AMR software stack personally — not just specifying it",
    "domains": ["Real-time motion & dispatch control (asyncio event loops, state machines)", "ROS 2 nodes, Nav2 behavior trees, SLAM tuning", "Fieldbus masters (pysoem/EtherCAT, Modbus TCP, CANopen)", "MQTT broker/topic design and multi-AGV coordination", "Web HMI + Flask/WebSocket backends", "Simulation & visualization tooling (pygame)", "Internal tooling / automation scripts (JS userscripts, CSV pipelines)"],
    "repos": ["agv-kim2a-controller", "iprime-presentation-tool", "pkb-bulk-fill.user.js"],
    "preferences": "Wants architecture-level reasoning before code; concurrency, failure modes, and recovery paths made explicit; Linux-native deployment (systemd, netplan) assumed"
  },

  "compliance_scope": {
    "directives": ["Machinery 2006/42/EC", "EMC", "Battery Regulation 2023/1542"],
    "standards": ["ISO 3691-4:2023", "EN ISO 13849-1", "IEC 62619"],
    "safety_target": "PL d / Cat 3 — SICK NanoScan3 + Flexi Soft FX3; TÜV SÜD voluntary assessment"
  },

  "tech_stack": {
    "software": ["Python (asyncio)", "ROS / ROS 2 Jazzy", "Nav2", "Cartographer", "pysoem", "Flask", "MQTT", "WebSocket", "pygame", "JavaScript (userscripts)", "Git / GitHub", "Bash / Ubuntu admin (systemd, netplan)", "Windows", "Excel"],
    "protocols": ["EtherCAT", "Modbus TCP", "CANopen", "MQTT", "Ethernet/Cat6"],
    "hardware": ["Mitsubishi PLCs & HMIs", "Industrial PCs (NEXCOM Neu-X102-N97, BKHD-1264-SFP)", "Keyence sensors", "SICK NanoScan3 / GLS6", "IDEC SE2L", "Xsens MTi-320 IMU", "Kinco iWMC wheel motors", "Leadshine EL7-ECN servo", "ez-Wheel SWD", "Moxa / TP-Link / Ubiquiti network gear", "LFP battery packs 51.2V 40–80Ah"],
    "components_sourced_from": ["Japan", "China", "Europe (SICK, Germany)"],
    "locally_made": ["Steel chassis", "Wiring", "Basic electrical"]
  },

  "company_context": {
    "automation_level": "High — in-house integration of 6–7 axis robots, conveyors, gantry, AGVs, robotic inspection",
    "agv_as_product": "IGP develops and sells AGV systems externally, not just internal use",
    "astra_group_advantage": "Internal credibility across Astra subsidiaries; strong integration knowledge of factory floor systems",
    "target_market": "Jabodetabek / West Java / Banten industrial corridor, 2026–2028; 5-ton line-following segment prioritized, Astra captive channel as primary moat",
    "competitive_pressure": "Chinese AMR/AGV vendors on landed cost"
  },

  "career_interests": {
    "direction": "Technical depth plus a bridge toward business development within Astra Otoparts",
    "credentials_considered": ["CMA (Certified Management Accountant)"]
  }
}
```

## Change log vs. previous version

| Area | Change |
|---|---|
| CE / compliance | Cut ~50%: directives 5→3, standards 7→3, `assessment_body` folded into `safety_target`, `solution_scope` CE string shortened |
| Software emphasis | New `software_development` section (domains, repos, working preferences) |
| Format rules | New `code_exception` — the no-code rule no longer suppresses code on actual software tasks |
| `current_work.primary` | Reworded so controls/software is named as the main personal workload |
| `technical_ownership` | Reordered software-first; "Control software architecture" and "Fleet & dispatch software" added |
| `active_projects` | `agv_controller` (PLC→asyncio migration) promoted to its own entry |
| `tech_stack.software` | +JavaScript, Git/GitHub, Bash/systemd/netplan |
