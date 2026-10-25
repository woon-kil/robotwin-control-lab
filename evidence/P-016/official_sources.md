# P-016 official sources

조회일: 2026-10-25. 본문 전문은 복제하지 않는다.

## 1차

- Omron, PMAC Software downloads. IDE 4.6.5.3, PDK 4.6.4.4, Simulator 1.2.1. 설치 전 Software License Agreement.  
  https://www.ia.omron.com/product/tool/pmac-software/
- Omron, PMAC Software history. IDE 4.6.5.3 (2026-01-19), IDE 4.6.4.23의 OPC UA / MQTT / Simulator 연결, IDE 4.6.3 이상의 Windows 11, Simulator 1.2.1과 1.2.0 기능.  
  https://www.ia.omron.com/product/tool/pmac-software/history.htm
- Alex Anikstein, Omron forum, 2025-04-14. IDE 4.6.4.23과 Simulator 1.2.0 공개. virtual PMAC, 실제 motor/I/O 없음, accessory module 없음.  
  https://forums.automation.omron.com/topic/10806-announcing-power-pmac-ide-464-power-pmac-simulator-120/
- Omron, Power PMAC IDE User Manual, Cat. No. O055-E-18. Windows 10, Windows 11. virtual machine RAM 문구. Script와 C, breakpoint.  
  https://www.omron-ap.co.nz/data_pdf/mnu/o055-e-18_power_pmac_ide.pdf?id=3725
- Omron / Delta Tau, Power PMAC Software Reference Manual, O015-E-01, 2023-05-19. `Sys.SimConfigOk` 항목. `EncTable` type 11은 virtual motor의 simulated servo loop를 허용한다고 적는다. `Motor[x].ActPos`, `Motor[x].DesPos` 항목이 있다.  
  https://files.omron.eu/downloads/latest/manual/en/o015_software_reference_manual_technical_manual_en.pdf?v=1

## Forum 답변

- steve.milici, 2025-04-15. CK3M project의 Simulator download segfault에 `Sys.SimConfigOk=1` 저장을 안내. 같은 실패가 남으면 technical support.  
  https://forums.automation.omron.com/topic/10807-download-process-failed-with-a-segmentation-fault-on-power-pmac-simulator/
- leandro.martins, 2025-04-24. encoder를 Simulator에서 고르지 못하는 설정에 `Sys.SimConfigOk=1` 저장을 안내.  
  https://forums.automation.omron.com/topic/10813-cant-add-encoder-in-simulator/
- MoMo, 같은 thread. Simulator는 virtual motor만 되고 physical motor 설정은 안 된다.
- Curt Wilson, 2014-03-31. `Sys.SimConfigOk=1`은 interface hardware가 줄거나 없는 시스템에 full configuration을 내려도 error를 내지 않게 한다. 없는 hardware로 보낸 setup 값은 backup에 없다.  
  https://forums.automation.omron.com/topic/4760-syssimconfigok/

직원 badge가 페이지에 없으면 직원이라고 적지 않았다. 위 계정은 해당 공개 thread의 답변이다.

## 쓰지 않은 것

GitHub의 SSH/gpascii library와 비공식 블로그는 Simulator 지원을 확정하는 근거로 쓰지 않았다. hardware용 SSH 관행을 Simulator 지원으로 올리지 않았다.
