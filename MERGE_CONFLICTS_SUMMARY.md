# Merge Conflicts Summary

This document outlines the merge conflicts that occurred when merging upstream/sunnypilot changes into the pr0-with-upstream branch.

## Files with Conflicts:
1. `.github/workflows/tests.yaml` - Both modified
2. `SConstruct` - Both modified
3. `common/pid.py` - Both modified
4. `opendbc_repo` - Submodule conflict (resolved)
5. `selfdrive/controls/controlsd.py` - Both modified
6. `selfdrive/controls/lib/drive_helpers.py` - Both modified
7. `selfdrive/pandad/tests/test_pandad.py` - Both modified
8. `selfdrive/ui/.gitignore` - Both modified
9. `selfdrive/ui/SConscript` - Both modified
10. `selfdrive/ui/main.cc` - Deleted by upstream
11. `selfdrive/ui/onroad/hud_renderer.py` - Both modified
12. `selfdrive/ui/sunnypilot/SConscript` - Deleted by upstream
13. `selfdrive/ui/sunnypilot/ui.cc` - Deleted by upstream
14. `selfdrive/ui/sunnypilot/ui.h` - Deleted by upstream
15. `selfdrive/ui/tests/test_runner.cc` - Deleted by upstream
16. `selfdrive/ui/tests/test_translations.cc` - Deleted by upstream
17. `selfdrive/ui/translations/main_de.ts` - Deleted by upstream
18. `selfdrive/ui/translations/main_fr.ts` - Deleted by upstream
19. `selfdrive/ui/translations/main_ko.ts` - Deleted by upstream
20. `selfdrive/ui/ui.cc` - Deleted by upstream
21. `selfdrive/ui/ui.h` - Deleted by upstream
22. `selfdrive/ui/ui.py` - Both modified
23. `selfdrive/ui/ui_state.py` - Both modified
24. `selfdrive/ui/update_translations.py` - Both modified
25. `system/manager/process_config.py` - Both modified
26. `system/ui/lib/application.py` - Both modified
27. `tools/cabana/SConscript` - Both modified
28. `tools/cabana/cabana.cc` - Both modified
29. `tools/cabana/chart/chart.cc` - Deleted by local
30. `tools/cabana/detailwidget.cc` - Deleted by local
31. `tools/cabana/detailwidget.h` - Deleted by local
32. `tools/cabana/mainwin.cc` - Deleted by local
33. `tools/cabana/signalview.cc` - Deleted by local
34. `tools/cabana/streams/routes.h` - Both modified
35. `tools/cabana/utils/api.cc` - Deleted by local
36. `tools/cabana/utils/api.h` - Deleted by local
37. `tools/cabana/utils/util.cc` - Both modified
38. `tools/cabana/utils/util.h` - Both modified
39. `tools/cabana/videowidget.cc` - Deleted by local
40. `tools/install_ubuntu_dependencies.sh` - Both modified
41. `uv.lock` - Both modified

## Approach for Resolution:
1. For files deleted by upstream, decisions need to be made about whether to keep local changes or follow upstream.
2. For files deleted by local, we should consider if these were Sunnypilot-specific features that may need to be preserved.
3. For files modified on both sides, manual conflict resolution is required.
4. Many changes in UI components likely reflect changes in the UI structure from the upstream.
5. The cabana tool changes represent significant UI restructuring from upstream.