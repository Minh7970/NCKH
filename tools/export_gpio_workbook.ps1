$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$outputPath = Join-Path $projectRoot 'docs\Bang_ket_noi_GPIO_ESP32.xlsx'

function ExcelColor([int]$r, [int]$g, [int]$b) {
    return $r + (256 * $g) + (65536 * $b)
}

function Set-CellTableStyle($sheet, [int]$headerRow, [int]$lastRow, [int]$lastColumn) {
    $header = $sheet.Range($sheet.Cells.Item($headerRow, 1), $sheet.Cells.Item($headerRow, $lastColumn))
    $header.Font.Bold = $true
    $header.Font.Color = ExcelColor 255 255 255
    $header.Interior.Color = ExcelColor 68 114 196
    $header.HorizontalAlignment = -4108
    $header.VerticalAlignment = -4108
    $header.WrapText = $true

    $table = $sheet.Range($sheet.Cells.Item($headerRow, 1), $sheet.Cells.Item($lastRow, $lastColumn))
    $table.Borders.LineStyle = 1
    $table.Borders.Weight = 2
    $table.VerticalAlignment = -4160
    $table.WrapText = $true
    $table.AutoFilter() | Out-Null
}

function Set-Title($sheet, [string]$title, [int]$lastColumn) {
    $sheet.Range($sheet.Cells.Item(1, 1), $sheet.Cells.Item(1, $lastColumn)).Merge()
    $titleCell = $sheet.Cells.Item(1, 1)
    $titleCell.Value2 = $title
    $titleCell.Font.Bold = $true
    $titleCell.Font.Size = 15
    $titleCell.Font.Color = ExcelColor 255 255 255
    $titleCell.Interior.Color = ExcelColor 31 78 121
    $titleCell.HorizontalAlignment = -4108
    $titleCell.VerticalAlignment = -4108
    $sheet.Rows.Item(1).RowHeight = 28
}

$gpioRows = @(
    @('STT','Linh kiện','Chân module','GPIO ESP32','Kiểu tín hiệu','Nguồn cấp','Trạng thái / ghi chú'),
    @(1,'L298N – động cơ trái','ENA','GPIO25','PWM output','Logic 3.3V','Tháo jumper ENA'),
    @(2,'L298N – động cơ trái','IN1','GPIO26','Digital output','Logic 3.3V','Điều khiển chiều'),
    @(3,'L298N – động cơ trái','IN2','GPIO27','Digital output','Logic 3.3V','Điều khiển chiều'),
    @(4,'L298N – động cơ phải','ENB','GPIO14','PWM output','Logic 3.3V','Tháo jumper ENB'),
    @(5,'L298N – động cơ phải','IN3','GPIO16','Digital output','Logic 3.3V','Điều khiển chiều'),
    @(6,'L298N – động cơ phải','IN4','GPIO17','Digital output','Logic 3.3V','Điều khiển chiều'),
    @(7,'Cảm biến lửa trái','AO','GPIO32','Analog input','3.3V','Đang sử dụng AO; không dùng DO'),
    @(8,'Cảm biến lửa phải','AO','GPIO33','Analog input','3.3V','Đang sử dụng AO; không dùng DO'),
    @(9,'MPU6050','SDA','GPIO23','I²C data','3.3V','Địa chỉ 0x68; AD0 nối GND'),
    @(10,'MPU6050','SCL','GPIO5','I²C clock','3.3V','Giữ xe đứng yên khi hiệu chuẩn'),
    @(11,'Radar HC-SR04 quay','TRIG','GPIO21','Digital output','5V','Tín hiệu phát siêu âm'),
    @(12,'Radar HC-SR04 quay','ECHO','GPIO22','Digital input','5V','Bắt buộc qua cầu phân áp xuống 3.3V'),
    @(13,'SA2/SRF05 phía trước','TRIG','GPIO4','Digital output','5V','Phát hiện vật cản khi xe đang chạy'),
    @(14,'SA2/SRF05 phía trước','ECHO','GPIO13','Digital input','5V','Bắt buộc qua cầu phân áp xuống 3.3V'),
    @(15,'Relay máy bơm','IN','GPIO18','Digital output','Theo module relay','Kích mức HIGH'),
    @(16,'Servo radar','Signal','GPIO19','PWM servo','Nguồn 5V ngoài','Không cấp nguồn từ ESP32'),
    @(17,'Servo cảm biến lửa trái','Signal','GPIO2','PWM servo','Nguồn 5V ngoài','GPIO boot; tránh kéo sai mức lúc khởi động'),
    @(18,'Servo cảm biến lửa phải','Signal','GPIO15','PWM servo','Nguồn 5V ngoài','GPIO boot; tránh kéo sai mức lúc khởi động'),
    @(19,'MQ-2','AO','GPIO35','Analog input','5V','Đang bật; AO phải qua cầu phân áp'),
    @(20,'MWIR confirmation','OUT','GPIO36','Digital input','Theo module','Đang tắt trong code'),
    @(21,'Servo vòi nước','Không sử dụng','—','—','—','Đã loại bỏ; vòi cố định trước mũi xe')
)

$powerRows = @(
    @('STT','Nhóm','Điểm đầu','Điểm cuối','Yêu cầu / ghi chú'),
    @(1,'Động cơ trái','L298N OUT1','Dây đen của hai motor trái','Hai motor đấu song song'),
    @(2,'Động cơ trái','L298N OUT2','Dây đỏ của hai motor trái','Theo chiều tiến của code hiện tại'),
    @(3,'Động cơ phải','L298N OUT3','Dây đen của hai motor phải','Hai motor đấu song song'),
    @(4,'Động cơ phải','L298N OUT4','Dây đỏ của hai motor phải','Theo chiều tiến của code hiện tại'),
    @(5,'Nguồn động cơ','Dương pin động cơ','L298N 12V/VIN','Không cấp motor từ ESP32'),
    @(6,'Nguồn động cơ','Âm pin động cơ','L298N GND','Nối chung GND ESP32'),
    @(7,'Nguồn servo','Nguồn 5V ngoài','Ba dây đỏ servo','Nguồn đủ dòng; không lấy từ chân 3V3'),
    @(8,'Mass servo','Âm nguồn servo','GND servo và ESP32','Bắt buộc nối chung mass'),
    @(9,'HC-SR04 ECHO','ECHO qua điện trở 1 kΩ','GPIO22; thêm 2 kΩ từ GPIO22 xuống GND','Hạ 5V xuống khoảng 3.3V'),
    @(10,'SA2 ECHO','ECHO qua điện trở 1 kΩ','GPIO13; thêm 2 kΩ từ GPIO13 xuống GND','Hạ 5V xuống khoảng 3.3V'),
    @(11,'MQ-2 AO','AO qua điện trở 10 kΩ','GPIO35; thêm 20 kΩ từ GPIO35 xuống GND','Không đưa AO 5V trực tiếp vào ESP32'),
    @(12,'Relay bơm','Dương nguồn bơm','COM relay','Nguồn bơm độc lập'),
    @(13,'Relay bơm','NO relay','Dương máy bơm','Bơm mặc định tắt'),
    @(14,'Máy bơm','Âm máy bơm','Âm nguồn bơm','Nên mắc diode chống ngược song song bơm'),
    @(15,'MPU6050','VCC / GND','3V3 / GND ESP32','AD0 nối GND; INT, XDA, XCL bỏ trống'),
    @(16,'Mass chung','ESP32, L298N, cảm biến, servo, MQ-2, relay','GND chung','Điều kiện bắt buộc để tín hiệu ổn định')
)

$summaryRows = @(
    @('GPIO','Chức năng','Trạng thái','Lưu ý'),
    @('GPIO2','Servo cảm biến lửa trái','Đang dùng','Chân boot'),
    @('GPIO4','SA2 TRIG','Đang dùng','Digital output'),
    @('GPIO5','MPU6050 SCL','Đang dùng','I²C clock; chân boot'),
    @('GPIO13','SA2 ECHO','Đang dùng','Qua cầu phân áp'),
    @('GPIO14','L298N ENB','Đang dùng','PWM động cơ phải'),
    @('GPIO15','Servo cảm biến lửa phải','Đang dùng','Chân boot'),
    @('GPIO16','L298N IN3','Đang dùng','Chiều động cơ phải'),
    @('GPIO17','L298N IN4','Đang dùng','Chiều động cơ phải'),
    @('GPIO18','Relay máy bơm','Đang dùng','Active HIGH'),
    @('GPIO19','Servo radar','Đang dùng','PWM servo'),
    @('GPIO21','Radar TRIG','Đang dùng','Digital output'),
    @('GPIO22','Radar ECHO','Đang dùng','Qua cầu phân áp'),
    @('GPIO23','MPU6050 SDA','Đang dùng','I²C data'),
    @('GPIO25','L298N ENA','Đang dùng','PWM động cơ trái'),
    @('GPIO26','L298N IN1','Đang dùng','Chiều động cơ trái'),
    @('GPIO27','L298N IN2','Đang dùng','Chiều động cơ trái'),
    @('GPIO32','Cảm biến lửa trái AO','Đang dùng','ADC'),
    @('GPIO33','Cảm biến lửa phải AO','Đang dùng','ADC'),
    @('GPIO35','MQ-2 AO','Đang dùng','Input-only; qua cầu phân áp'),
    @('GPIO36','MWIR OUT','Đang tắt','Input-only; không cần nối')
)

$excel = $null
$workbook = $null
try {
    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = $false
    $excel.DisplayAlerts = $false
    $workbook = $excel.Workbooks.Add()

    while ($workbook.Worksheets.Count -lt 3) {
        $null = $workbook.Worksheets.Add()
    }
    while ($workbook.Worksheets.Count -gt 3) {
        $workbook.Worksheets.Item($workbook.Worksheets.Count).Delete()
    }

    $sheet1 = $workbook.Worksheets.Item(1)
    $sheet1.Name = 'GPIO ESP32'
    Set-Title $sheet1 'BẢNG KẾT NỐI GPIO – ROBOT CHỮA CHÁY ESP32 DEVKIT V1' 7
    for ($r = 0; $r -lt $gpioRows.Count; $r++) {
        for ($c = 0; $c -lt $gpioRows[$r].Count; $c++) {
            $sheet1.Cells.Item($r + 2, $c + 1).Value2 = [string]$gpioRows[$r][$c]
        }
    }
    $gpioLastRow = $gpioRows.Count + 1
    Set-CellTableStyle $sheet1 2 $gpioLastRow 7
    $sheet1.Range('A3:A' + $gpioLastRow).HorizontalAlignment = -4108
    $sheet1.Range('D3:F' + $gpioLastRow).HorizontalAlignment = -4108
    $sheet1.Columns.Item(1).ColumnWidth = 6
    $sheet1.Columns.Item(2).ColumnWidth = 28
    $sheet1.Columns.Item(3).ColumnWidth = 18
    $sheet1.Columns.Item(4).ColumnWidth = 15
    $sheet1.Columns.Item(5).ColumnWidth = 18
    $sheet1.Columns.Item(6).ColumnWidth = 18
    $sheet1.Columns.Item(7).ColumnWidth = 39
    $sheet1.Rows.Item('2:' + $gpioLastRow).AutoFit() | Out-Null
    $sheet1.Application.ActiveWindow.SplitRow = 2
    $sheet1.Application.ActiveWindow.FreezePanes = $true

    $sheet2 = $workbook.Worksheets.Item(2)
    $sheet2.Name = 'Nguồn & động lực'
    Set-Title $sheet2 'BẢNG ĐẤU NGUỒN, ĐỘNG CƠ VÀ MẠCH BẢO VỆ' 5
    for ($r = 0; $r -lt $powerRows.Count; $r++) {
        for ($c = 0; $c -lt $powerRows[$r].Count; $c++) {
            $sheet2.Cells.Item($r + 2, $c + 1).Value2 = [string]$powerRows[$r][$c]
        }
    }
    $powerLastRow = $powerRows.Count + 1
    Set-CellTableStyle $sheet2 2 $powerLastRow 5
    $sheet2.Range('A3:A' + $powerLastRow).HorizontalAlignment = -4108
    $sheet2.Columns.Item(1).ColumnWidth = 6
    $sheet2.Columns.Item(2).ColumnWidth = 21
    $sheet2.Columns.Item(3).ColumnWidth = 30
    $sheet2.Columns.Item(4).ColumnWidth = 40
    $sheet2.Columns.Item(5).ColumnWidth = 48
    $sheet2.Rows.Item('2:' + $powerLastRow).AutoFit() | Out-Null

    $sheet3 = $workbook.Worksheets.Item(3)
    $sheet3.Name = 'Tóm tắt chân'
    Set-Title $sheet3 'TÓM TẮT GPIO ĐÃ SỬ DỤNG' 4
    for ($r = 0; $r -lt $summaryRows.Count; $r++) {
        for ($c = 0; $c -lt $summaryRows[$r].Count; $c++) {
            $sheet3.Cells.Item($r + 2, $c + 1).Value2 = [string]$summaryRows[$r][$c]
        }
    }
    $summaryLastRow = $summaryRows.Count + 1
    Set-CellTableStyle $sheet3 2 $summaryLastRow 4
    $sheet3.Range('A3:A' + $summaryLastRow).HorizontalAlignment = -4108
    $sheet3.Columns.Item(1).ColumnWidth = 14
    $sheet3.Columns.Item(2).ColumnWidth = 33
    $sheet3.Columns.Item(3).ColumnWidth = 17
    $sheet3.Columns.Item(4).ColumnWidth = 40
    $sheet3.Rows.Item('2:' + $summaryLastRow).AutoFit() | Out-Null

    foreach ($sheet in @($sheet1, $sheet2, $sheet3)) {
        $sheet.PageSetup.Orientation = 2
        $sheet.PageSetup.Zoom = $false
        $sheet.PageSetup.FitToPagesWide = 1
        $sheet.PageSetup.FitToPagesTall = 1
        $sheet.PageSetup.LeftMargin = $excel.InchesToPoints(0.3)
        $sheet.PageSetup.RightMargin = $excel.InchesToPoints(0.3)
        $sheet.PageSetup.TopMargin = $excel.InchesToPoints(0.5)
        $sheet.PageSetup.BottomMargin = $excel.InchesToPoints(0.5)
        $sheet.PageSetup.CenterFooter = 'Nguồn cấu hình: lib/RobotConfig/RobotConfig.h – 08/10/2026'
    }

    $workbook.SaveAs($outputPath, 51)
    $workbook.Close($true)
    $excel.Quit()
}
finally {
    if ($workbook -ne $null) {
        try { [void][Runtime.InteropServices.Marshal]::ReleaseComObject($workbook) } catch {}
    }
    if ($excel -ne $null) {
        try { [void][Runtime.InteropServices.Marshal]::ReleaseComObject($excel) } catch {}
    }
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}

Write-Output $outputPath
