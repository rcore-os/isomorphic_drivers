use alloc::vec::Vec;

pub trait Provider: Sync + Send {
    /// Get page size
    fn get_page_size(&self) -> usize;

    // Translate virtual address to physical address
    fn translate_va(&self, va: usize) -> usize;

    // Bulk translate virtual addresses to physical addresses for performance
    fn translate_vas(&self, vas: &[usize]) -> Vec<usize>;
}