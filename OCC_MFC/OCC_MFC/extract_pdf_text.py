import fitz
doc = fitz.open("OCC_MFC/weld_method_paper.pdf")
with open("OCC_MFC/paper_full_text.txt", "w", encoding="utf-8") as f:
    for i, page in enumerate(doc):
        text = page.get_text("text")
        f.write(f"\n===== Page {i+1} =====\n")
        f.write(text)
        f.write("\n")
print(f"Extracted {doc.page_count} pages")
doc.close()
