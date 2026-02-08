import os
import re
from fpdf import FPDF

# List of files to include in order
docs_files = [
    "README.md",
    "AGENT_CONTEXT.md",
    "retoma.md",
    "docs/project_Status_executive_summary.md",
    "docs/design/system_specification.md",
    "docs/hardware/hardware_compatibility_report.md",
    "docs/hardware/hid_analysis.md",
    "docs/hardware/lcd_config.md",
    "docs/protocols/winwing_mcdu/overview.md",
    "docs/protocols/winwing_mcdu/protocol.md",
    "docs/protocols/minifcu/protocol.md",
    "docs/protocols/espnow/latency_optimization.md",
    "docs/protocols/thrustmaster/methodology.md",
    "docs/protocols/thrustmaster/quadrant_hid.md",
    "docs/protocols/thrustmaster/sidestick_hid.md",
    "docs/testing/test_runner.md",
    "firmware/coordinator/README.md",
]

base_dir = r"d:\simbridge\github\opencockpit-wireless-bus"
output_md = os.path.join(base_dir, "consolidated_docs.md")
output_pdf = os.path.join(base_dir, "consolidated_docs.pdf")

class DOCPDF(FPDF):
    def header(self):
        self.set_font('helvetica', 'B', 10)
        self.cell(0, 10, 'Opencockpit Wireless Bus Documentation', 0, 1, 'C')
        self.ln(2)

    def footer(self):
        self.set_y(-15)
        self.set_font('helvetica', 'I', 8)
        self.cell(0, 10, f'Page {self.page_no()}', 0, 0, 'C')

def clean_text(text):
    # Remote non-latin characters and unsupported symbols for basic Helvetica
    # Also strip code blocks for cleaner text PDF if they cause issues, but for now just encode
    return text.encode('latin-1', 'replace').decode('latin-1')

def generate_pdf():
    pdf = DOCPDF()
    pdf.set_auto_page_break(auto=True, margin=15)
    
    # Title Page
    pdf.add_page()
    pdf.set_font("helvetica", "B", 24)
    pdf.ln(60)
    pdf.cell(0, 20, "Opencockpit Wireless Bus", ln=True, align="C")
    pdf.set_font("helvetica", "B", 18)
    pdf.cell(0, 10, "Consolidated Documentation", ln=True, align="C")
    pdf.ln(20)
    pdf.set_font("helvetica", "", 12)
    pdf.multi_cell(0, 10, "This document contains a consolidation of all Markdown documentation files found in the repository.", align="C")
    pdf.add_page()

    consolidated_content = "# Opencockpit Wireless Bus Documentation\n\n"

    for relative_path in docs_files:
        full_path = os.path.join(base_dir, relative_path.replace("/", os.sep))
        if os.path.exists(full_path):
            title = relative_path.replace("/", " > ")
            
            # Write Header for Document
            pdf.set_font("helvetica", "B", 14)
            pdf.set_fill_color(220, 220, 220)
            pdf.cell(0, 10, f"Source: {title}", ln=True, fill=True)
            pdf.ln(2)
            
            with open(full_path, "r", encoding="utf-8") as infile:
                content = infile.read()
                
                # Simplified rendering: just multi_cell the text
                # This avoids HTML parsing issues
                pdf.set_font("helvetica", "", 9)
                pdf.multi_cell(0, 5, clean_text(content))
                
                pdf.add_page()
                
                # Also build consolidated MD
                consolidated_content += f"\n\n---\n\n# Document: {title}\n\n"
                consolidated_content += content + "\n"
        else:
            print(f"Warning: {full_path} not found.")

    pdf.output(output_pdf)
    
    with open(output_md, "w", encoding="utf-8") as f:
        f.write(consolidated_content)

if __name__ == "__main__":
    generate_pdf()
    print(f"Successfully generated PDF: {output_pdf}")
    print(f"Successfully generated MD: {output_md}")
