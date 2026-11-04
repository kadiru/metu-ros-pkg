# Research sources and visual attribution

These landing pages document two historical stacks in `kadiru/metu-ros-pkg`. The source snapshot reviewed for the documentation is [`315856b179f34c955bd50ef512a54c3cdf160966`](https://github.com/kadiru/metu-ros-pkg/commit/315856b179f34c955bd50ef512a54c3cdf160966). The publication dates describe the research; the repository import commit is from 2016.

## Research sources

| ID | Source | What it supports |
| --- | --- | --- |
| P1 | Uyanık et al. (2013), [Learning Social Affordances and Using Them for Planning](https://escholarship.org/uc/item/9cj412wg) | Entity–behavior–effect representation; object and human interactions; SVM effect prediction; effect prototypes; depth-three breadth-first planning; observation and replanning |
| P2 | Yürüten et al. (2012), [Learning Adjectives and Nouns from Affordances on the iCub Humanoid Robot](https://link.springer.com/chapter/10.1007/978-3-642-33093-3_33) | A48/A8/appearance comparison; six behaviors and eight effects; adjective and noun results; original iCub video |
| P3 | Uyanık et al. (2012), [Towards Context-aware Adjective Learning](https://kovan.ceng.metu.edu.tr/~sinan/publications/icra_hri_12.pdf) | Relative perceptual features and affordance-based contextual adjective learning |
| P4 | Uyanık et al. (2011), [RoboTurk 2011 Team Description](https://download.tigers-mannheim.de/tdps/2011_TDP_RoboTurk.pdf) | Five robots per team; four-omniwheel mechanics; control electronics; ROS architecture; Webots; development-stage limitations |
| P5 | Kadir Fırat Uyanık, [A study on Artificial Potential Fields](https://www.academia.edu/2007421/A_study_on_Artificial_Potential_Fields) — author-supplied course technical report, 6 pages | Potential-field experiments; Table I; simulation architecture; Section VI video links |
| P6 | Kadir Fırat Uyanık, [A comprehensive study on path planning algorithms in multi-agent robot soccer domain](https://www.scribd.com/document/55167175/A-comprehensive-study-on-path-planning-algorithms-in-multi-agent-robot-soccer-domain) — author-supplied EE586 progress report, 7 pages | Preliminary APF results and explicitly unfinished visibility-graph/RRT comparison |

The two course reports were checked against the original author-supplied PDFs. Their titles and authorship also match the public document pages linked above. Those links open Academia.edu and Scribd, not direct PDF files hosted in this repository; downloading may require an account with the respective host. Scribd displays the alternative platform title “Path Planning in Multi-Agent Robotics”; the title printed in the report is retained here. No venue, publication year or DOI has been guessed, and the original PDF files are not bundled in this repository.

## Visuals

The green **[affordance-learning banner](assets/affordance-hero.svg)** reuses the tabletop and perception/action-loop illustration from the author's research landing-page draft. It is an explanatory illustration, not experimental footage or measured data.

**[Social architecture](assets/social-architecture.svg)** is a redrawn conceptual overview of P1, especially the methodology and Figures 3–4. Related code links identify inspected packages; they do not certify an exact reproduction of the paper.

**[Language architecture](assets/language-architecture.svg)** follows P2, Figure 1 and Sections 3.3–3.5. Verb labels describe effects; the learned object labels in that paper are **adjectives and nouns**.

**[RoboTurk architecture](assets/roboturk-architecture.svg)** draws on P4, Sections 2–3, and P5, Section III. It describes the proposed/experimental system and preserves the distinction between completed modules and unfinished higher-level game planning.

SVGs render as static images in the READMEs. Package and paper links are repeated in the surrounding Markdown so navigation does not depend on clickable regions inside an image. These README pages do not execute the original site's JavaScript illustrations.

## Code-reading boundaries

Representative source, build and launch files were inspected; the repository was not compiled or executed. In particular:

- `al_prediction/effect_predictor.cpp` exposes learned-effect and behavior-by-effect services and loads behavior-specific SVM models.
- `al_memory/src/Memory.cpp` connects scene/entity queries to perception and feature-vector publication.
- `al_hri_demo/hri_main.cpp` connects ROS transforms and speech to YARP gaze control.
- `ssl_path_planner_/src/SSLPathPlanner.cpp` contains substantial geometry, planning and robot-execution code. Its pRRT parameters and 100 ms planning budget are configuration, not benchmark measurements.
- The non-suffixed `ssl_path_planner` package is a distinct scaffold, not evidence of a completed planner.

The papers and code are separate kinds of evidence. Reported results are not upgraded by the presence of related source files. Code and bundled third-party libraries retain their original, file-specific licenses; this documentation does not relicense them.

[Affordance-learning stack](../stacks/affordance_learning/README.md) · [Robot-soccer stack](../stacks/robocup_ssl/README.md) · [Repository overview](../README.md)
